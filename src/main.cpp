#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <vector>
#include <utils.hpp>
#include <pose.hpp>

//"http://10.120.19.36:4747/video",cv::CAP_FFMPEG
// Usage : ./CoursOpenCv [video] [camera.yaml]
int main(int argc, char* argv[]) {
  std::string videoPath = argc > 1 ? argv[1] : "../videos/prof.mp4";
  std::string camPath = argc > 2 ? argv[2] : "../input_camera.yaml";

  cv::VideoCapture cap(videoPath);
  if (!cap.isOpened()) {
    std::cerr << "il faut " << videoPath << " le sang" << std::endl;
    return 1;
  }

  cv::Mat frame;
  cap.read(frame);
  if (frame.rows > LARGECONST || frame.cols > LARGECONST)
    cv::resize(frame, frame, cv::Size(), REDUCTIONFACTOR, REDUCTIONFACTOR);

  // Taille des images APRÈS réduction : c'est elle que K doit décrire.
  cv::Size frameSize = frame.size();
  CameraParams cam;
  if (!loadCameraParams(camPath, cam, frameSize)) return 1;
  std::cout << "K =\n" << cam.K << "\ndist = " << cam.dist << std::endl;
  std::cout << "Video Params : \n "
    << "width : " << frame.cols << "\n "
    << "hight : " << frame.rows << "\n "
    << std::endl;


  double fps = cap.get(cv::CAP_PROP_FPS);
  int delay = fps > 0 ? (int)(1000.0 / fps) : 30;
  cv::namedWindow("A_good_name", cv::WINDOW_AUTOSIZE);

  // Fenêtre OpenGL à côté de la vidéo (tout le code est dans glCube.cpp)
  if (!glCubeInit(cam, frameSize)) return 1;

  std::vector<std::vector<cv::Point>> pointHist;


  std::vector<cv::Point> quad;
  std::vector<cv::Point> prevQuad;
  while (true) {
    if (!cap.read(frame)) {
      cap.set(cv::CAP_PROP_POS_FRAMES, 0);
      if (!cap.read(frame)) break;
    }

    cv::Mat small;
    if (frame.rows > LARGECONST || frame.cols > LARGECONST){
      cv::resize(frame, frame, cv::Size(), REDUCTIONFACTOR, REDUCTIONFACTOR);
    }

    bool found = false;  // feuille trouvée sur cette image ?
    cv::Mat rvec, tvec;  // sa pose (si trouvée)

    // Croissance de région
    cv::Point seed(frame.cols / 2, frame.rows / 2);
    cv::Mat black = growRegion(frame, seed, 30, 8);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(black, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (!contours.empty()) {
      auto biggest = *std::max_element(contours.begin(), contours.end(),
          [](auto& a, auto& b){ return cv::contourArea(a) < cv::contourArea(b); });
      std::vector<cv::Point> hull, quad;
      cv::convexHull(biggest, hull);
      double peri = cv::arcLength(hull, true);
      for (double eps = 0.02; eps <= 0.06 && quad.size() != 4; eps += 0.01)
        cv::approxPolyDP(hull, quad, eps * peri, true);
       
      cv::polylines(frame, quad, true, {0, 255, 255}, 2);  // un seul contour : polylines

      if (quad.size() == 4) {
        // 1) Ordre cohérent des coins (doit correspondre aux points 3D)
        std::vector<cv::Point2f> corners = orderCorners(quad);

        // 2) Raffinement sub-pixel sur l'image en niveaux de gris
        cv::Mat gray = toGrayScale(frame);
        cv::cornerSubPix(gray, corners, cv::Size(7, 7), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));

        // 3) Pose de la feuille
        double err = 0;
        found = estimatePose(corners, cam, rvec, tvec, &err);
        if (found) {
          drawCube(frame, cam, rvec, tvec, 100.f);
          cv::putText(frame,
              cv::format("dist=%.0f mm  reproj=%.2f px", cv::norm(tvec), err),
              {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 255}, 2);
        }
        for (int i = 0; i < 4; ++i) {
          cv::circle(frame, corners[i], 1, cv::Scalar(0, 0, 255), cv::FILLED);
          cv::putText(frame, std::to_string(i), corners[i] + cv::Point2f(8, -8),
              cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 0, 255}, 2);
        }
      }
    }
    prevQuad = quad;
    quad.clear();

    imshow("mask", black);
    imshow("A_good_name", frame);
    if (!glCubeDraw(found, rvec, tvec)) break;  // même pose, dessinée par OpenGL

    int key = cv::waitKey(delay);
    if (key == 27 || key == 'q') break;
    if (key == ' ') cv::waitKey(0);  // pause
  }
  glCubeClose();
  return 0;
}
