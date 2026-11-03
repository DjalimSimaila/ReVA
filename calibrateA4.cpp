
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <queue>
#include <string>
#include <vector>

namespace {

// ------------------------------------------------------------- Réglages
constexpr float  SHEET_W = 297.f, SHEET_H = 210.f;  ///< feuille A4 en mm
constexpr int    MAX_VIEWS = 120;                    ///< images utilisées au maximum
constexpr int    MIN_VIEWS = 10;                    ///< en dessous : pas fiable
constexpr double MIN_AREA_RATIO = 0.03;             ///< feuille >= 3 % de l'image
constexpr int    SAT_TOL = 30;                      ///< croissance de région : saturation
constexpr int    STEP = 8;                          ///< croissance de région : écart entre voisins
constexpr double INITIAL_HFOV_DEG = 70.0;           ///< point de départ de la focale

// ------------------------------------------------------------- Détection
/// Croissance de région en HSV à partir d'une graine 
cv::Mat growRegion(const cv::Mat& image, cv::Point seed) {
  cv::Mat img;
  cv::GaussianBlur(image, img, cv::Size(5, 5), 0);
  cv::cvtColor(img, img, cv::COLOR_BGR2HSV);
  cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC1);
  if (seed.x < 0 || seed.y < 0 || seed.x >= img.cols || seed.y >= img.rows) return mask;

  const cv::Vec3b ref = img.at<cv::Vec3b>(seed);
  std::queue<cv::Point> todo;
  todo.push(seed);
  mask.at<uchar>(seed) = 255;
  const cv::Point dirs[4] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  while (!todo.empty()) {
    cv::Point p = todo.front();
    todo.pop();
    const cv::Vec3b cp = img.at<cv::Vec3b>(p);
    for (const cv::Point& d : dirs) {
      cv::Point q = p + d;
      if (q.x < 0 || q.y < 0 || q.x >= img.cols || q.y >= img.rows || mask.at<uchar>(q)) continue;
      const cv::Vec3b c = img.at<cv::Vec3b>(q);
      bool peuColore   = std::abs(c[1] - ref[1]) <= SAT_TOL;
      bool pasTropNoir = c[2] >= ref[2] / 2;
      bool continu     = std::abs(c[1] - cp[1]) <= STEP && std::abs(c[2] - cp[2]) <= STEP;
      if (peuColore && pasTropNoir && continu) {
        mask.at<uchar>(q) = 255;
        todo.push(q);
      }
    }
  }
  return mask;
}

std::vector<cv::Point2f> orderCorners(const std::vector<cv::Point>& quad) {
  std::vector<cv::Point2f> pts(quad.begin(), quad.end());
  cv::Point2f center = (pts[0] + pts[1] + pts[2] + pts[3]) * 0.25f;
  std::sort(pts.begin(), pts.end(), [&](cv::Point2f a, cv::Point2f b) {
    return std::atan2(a.y - center.y, a.x - center.x) < std::atan2(b.y - center.y, b.x - center.x);
  });
  auto topLeft = std::min_element(pts.begin(), pts.end(),
      [](cv::Point2f a, cv::Point2f b) { return a.x + a.y < b.x + b.y; });
  std::rotate(pts.begin(), topLeft, pts.end());
  double side01 = cv::norm(pts[1] - pts[0]) + cv::norm(pts[3] - pts[2]);
  double side12 = cv::norm(pts[2] - pts[1]) + cv::norm(pts[0] - pts[3]);
  if (side12 > side01) std::rotate(pts.begin(), pts.begin() + 1, pts.end());
  return pts;
}

std::vector<cv::Point3f> a4ObjectPoints() {
  return {{-SHEET_W / 2, SHEET_H / 2, 0}, {SHEET_W / 2, SHEET_H / 2, 0},
          {SHEET_W / 2, -SHEET_H / 2, 0}, {-SHEET_W / 2, -SHEET_H / 2, 0}};
}

/// Détecte les 4 coins de la feuille dans une image.
bool detectCorners(const cv::Mat& frame, cv::Point seed, std::vector<cv::Point2f>& corners) {
  cv::Mat mask = growRegion(frame, seed);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if (contours.empty()) return false;
  auto biggest = *std::max_element(contours.begin(), contours.end(),
      [](const auto& a, const auto& b) { return cv::contourArea(a) < cv::contourArea(b); });
  if (cv::contourArea(biggest) < MIN_AREA_RATIO * frame.total()) return false;

  std::vector<cv::Point> hull, quad;
  cv::convexHull(biggest, hull);
  double peri = cv::arcLength(hull, true);
  for (double eps = 0.02; eps <= 0.06 && quad.size() != 4; eps += 0.01)
    cv::approxPolyDP(hull, quad, eps * peri, true);
  if (quad.size() != 4) return false;

  // Feuille qui touche le bord de l'image = coin coupé = coin faux : on ignore.
  for (const cv::Point& p : quad)
    if (p.x < 3 || p.y < 3 || p.x > frame.cols - 4 || p.y > frame.rows - 4) return false;

  corners = orderCorners(quad);
  cv::Mat gray;
  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  cv::cornerSubPix(gray, corners, {7, 7}, {-1, -1},
      cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));
  return true;
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc < 3) {
    std::cerr << "Usage : " << argv[0] << " <video> <sortie.yaml> [--show]" << std::endl;
    return 1;
  }
  const std::string videoPath = argv[1], outPath = argv[2];
  const bool show = argc > 3 && std::string(argv[3]) == "--show";

  cv::VideoCapture cap(videoPath);
  if (!cap.isOpened()) {
    std::cerr << "Impossible d'ouvrir " << videoPath << std::endl;
    return 1;
  }
  const cv::Size size((int)cap.get(cv::CAP_PROP_FRAME_WIDTH), (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT));
  const int total = (int)cap.get(cv::CAP_PROP_FRAME_COUNT);
  const int every = std::max(1, total / (MAX_VIEWS * 2));  // répartir les vues sur toute la vidéo

  // 1) Récolter les coins de la feuille sur la vidéo
  std::vector<std::vector<cv::Point2f>> imagePoints;
  const cv::Point center(size.width / 2, size.height / 2);
  cv::Point seed = center;
  cv::Mat frame;
  for (int i = 0; cap.read(frame) && (int)imagePoints.size() < MAX_VIEWS; ++i) {
    std::vector<cv::Point2f> corners;
    bool ok = detectCorners(frame, seed, corners);
    // La graine suit la feuille : centre de la feuille trouvée, sinon centre de l'image.
    seed = ok ? cv::Point((corners[0] + corners[1] + corners[2] + corners[3]) * 0.25f) : center;
    if (ok && i % every == 0) imagePoints.push_back(corners);

    if (show) {
      if (ok)
        for (int k = 0; k < 4; ++k) cv::line(frame, corners[k], corners[(k + 1) % 4], {0, 255, 0}, 2);
      cv::putText(frame, cv::format("vues : %zu", imagePoints.size()), {10, 30},
                  cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 255}, 2);
      cv::imshow("calibration A4", frame);
      if (cv::waitKey(1) == 27) break;  // Echap : arrêter la lecture et calibrer
    }
  }
  std::cout << imagePoints.size() << " vues de la feuille retenues" << std::endl;
  if ((int)imagePoints.size() < MIN_VIEWS) {
    std::cerr << "Pas assez de vues (" << MIN_VIEWS << " minimum) : la feuille doit etre "
                 "entiere a l'image et bien visible." << std::endl;
    return 1;
  }

  double f0 = 0.5 * size.width / std::tan(0.5 * INITIAL_HFOV_DEG * CV_PI / 180.0);
  cv::Mat K = (cv::Mat_<double>(3, 3) << f0, 0, size.width / 2.0, 0, f0, size.height / 2.0, 0, 0, 1);
  cv::Mat dist = cv::Mat::zeros(1, 5, CV_64F);
  std::vector<std::vector<cv::Point3f>> objectPoints(imagePoints.size(), a4ObjectPoints());
  std::vector<cv::Mat> rvecs, tvecs;
  const int flags = cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_FIX_PRINCIPAL_POINT |
                    cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST |
                    cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3;
  double rms = cv::calibrateCamera(objectPoints, imagePoints, size, K, dist, rvecs, tvecs, flags);

  double f = K.at<double>(0, 0);
  double hfov = 2.0 * std::atan(0.5 * size.width / f) * 180.0 / CV_PI;
  std::cout << "Resolution : " << size.width << "x" << size.height << "\n"
            << "Focale : " << f << " px  (champ de vision horizontal : " << hfov << " deg)\n"
            << "Erreur de reprojection RMS : " << rms << " px "
            << (rms < 1.0 ? "(bien)" : rms < 2.0 ? "(correct)" : "(a refaire : inclinez plus la feuille)")
            << std::endl;
  if (hfov < 30 || hfov > 120)
    std::cerr << "Attention : champ de vision peu realiste, la video manque sans doute de vues inclinees."
              << std::endl;

  cv::FileStorage fs(outPath, cv::FileStorage::WRITE);
  if (!fs.isOpened()) {
    std::cerr << "Impossible d'ecrire " << outPath << std::endl;
    return 1;
  }
  fs << "camera_matrix" << K << "distortion_coefficients" << dist;
  fs << "image_width" << size.width << "image_height" << size.height;
  fs << "rms_error" << rms << "views" << (int)imagePoints.size();
  std::cout << "Fichier ecrit : " << outPath << std::endl;
  return 0;
}
