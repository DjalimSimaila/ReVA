#include <pose.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <opencv2/imgproc.hpp>
#if CV_VERSION_MAJOR >= 5
#include <opencv2/3d.hpp>
#else
#include <opencv2/calib3d.hpp>
#endif

bool loadCameraParams(const std::string& path, CameraParams& cam, cv::Size frameSize) {
  cv::FileStorage fs(path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "Impossible d'ouvrir " << path << std::endl;
    return false;
  }
  fs["camera_matrix"] >> cam.K;
  fs["distortion_coefficients"] >> cam.dist;
  if (cam.K.empty()) return false;
  if (cam.dist.empty()) cam.dist = cv::Mat::zeros(1, 5, CV_64F);
  cam.K.convertTo(cam.K, CV_64F);
  cam.dist.convertTo(cam.dist, CV_64F);
  return true;
}

std::vector<cv::Point2f> orderCorners(const std::vector<cv::Point>& quad) {
  std::vector<cv::Point2f> pts(quad.begin(), quad.end());
  cv::Point2f center = (pts[0] + pts[1] + pts[2] + pts[3]) * 0.25f;

  // Tri par angle autour du centre : sens horaire à l'écran.
  std::sort(pts.begin(), pts.end(), [&](cv::Point2f a, cv::Point2f b) {
    return std::atan2(a.y - center.y, a.x - center.x) <
           std::atan2(b.y - center.y, b.x - center.x);
  });

  // Premier coin = le plus en haut à gauche.
  auto topLeft = std::min_element(pts.begin(), pts.end(), [](cv::Point2f a, cv::Point2f b) {
    return a.x + a.y < b.x + b.y;
  });
  std::rotate(pts.begin(), topLeft, pts.end());

  // L'arête 0->1 doit être un grand côté (on compare les paires de côtés opposés).
  double side01 = cv::norm(pts[1] - pts[0]) + cv::norm(pts[3] - pts[2]);
  double side12 = cv::norm(pts[2] - pts[1]) + cv::norm(pts[0] - pts[3]);
  if (side12 > side01) std::rotate(pts.begin(), pts.begin() + 1, pts.end());
  return pts;
}

std::vector<cv::Point3f> a4ObjectPoints(float w, float h) {
  // Même ordre que orderCorners.
  return {
      {-w / 2,  h / 2, 0},
      { w / 2,  h / 2, 0},
      { w / 2, -h / 2, 0},
      {-w / 2, -h / 2, 0},
  };
}

bool estimatePose(const std::vector<cv::Point2f>& imgPts, const CameraParams& cam,
                  cv::Mat& rvec, cv::Mat& tvec, double* reprojErr) {
  if (imgPts.size() != 4) return false;
  std::vector<cv::Point3f> objPts = a4ObjectPoints();

  // IPPE : solveur pour cibles planes.
  if (!cv::solvePnP(objPts, imgPts, cam.K, cam.dist, rvec, tvec, false, cv::SOLVEPNP_IPPE))
    return false;

  return tvec.at<double>(2) > 0;  // la feuille doit être devant la caméra
}

void drawBase(cv::Mat& img, const CameraParams& cam, const cv::Mat& rvec,
              const cv::Mat& tvec, float side) {
  float h = side / 2;
  std::vector<cv::Point3f> base = {{-h, -h, 0}, {h, -h, 0}, {h, h, 0}, {-h, h, 0}};
  std::vector<cv::Point2f> p;
  cv::projectPoints(base, rvec, tvec, cam.K, cam.dist, p);

  for (int i = 0; i < 4; ++i)
    cv::line(img, p[i], p[(i + 1) % 4], {200, 0, 0}, 2, cv::LINE_AA);
}
