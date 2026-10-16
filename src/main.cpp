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

// Usage : ./CoursOpenCv [video] [camera.yaml]
int main(int argc, char* argv[]) {
  std::string videoPath = argc > 1 ? argv[1] : "../b.mp4";
  std::string camPath = argc > 2 ? argv[2] : "../input_camera.yaml";

  cv::VideoCapture cap(videoPath);
  if (!cap.isOpened()) {
    std::cerr << "il faut " << videoPath << " le sang" << std::endl;
    return 1;
  }

  cv::Size frameSize((int)cap.get(cv::CAP_PROP_FRAME_WIDTH),
                     (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT));
  CameraParams cam;
  if (!loadCameraParams(camPath, cam, frameSize)) return 1;
  std::cout << "K =\n" << cam.K << "\ndist = " << cam.dist << std::endl;

  double fps = cap.get(cv::CAP_PROP_FPS);
  int delay = fps > 0 ? (int)(1000.0 / fps) : 30;
  cv::namedWindow("A_good_name", cv::WINDOW_AUTOSIZE);

  while (true) {
    cv::Mat frame;
    if (!cap.read(frame)) {
      cap.set(cv::CAP_PROP_POS_FRAMES, 0);
      if (!cap.read(frame)) break;
    }

    // Croissance de région
    cv::Point seed(frame.cols / 2, frame.rows / 2);
    cv::Mat black = growRegion(frame, seed, 30, 8);
    
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(black, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (!contours.empty()) {
      
      auto biggest = *std::max_element(contours.begin(), contours.end(),
          [](auto& a, auto& b){ return cv::contourArea(a) < cv::contourArea(b); });
      
      std::vector<cv::Point> quad;
      double peri = cv::arcLength(biggest, true);
      for (double eps = 0.02; eps <= 0.06 && quad.size() != 4; eps += 0.01)
        cv::approxPolyDP(biggest, quad, eps * peri, true);

      if (quad.size() == 4) {
        // 1) Ordre cohérent des coins (doit correspondre aux points 3D)
        std::vector<cv::Point2f> corners = orderCorners(quad);

        // 2) Pose de la feuille
        cv::Mat rvec, tvec;
        double err = 0;
        if (estimatePose(corners, cam, rvec, tvec, &err)) {
          drawBase(frame, cam, rvec, tvec, 100.f,{0,0,255});
        }
        for (int i = 0; i < 4; ++i) {
          cv::circle(frame, corners[i], 1, cv::Scalar(0, 0, 255), cv::FILLED);
          cv::putText(frame, std::to_string(i), corners[i] + cv::Point2f(8, -8),
                      cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 0, 255}, 2);
        }
      }
    }

    imshow("mask", black);
    imshow("A_good_name", frame);
    int key = cv::waitKey(delay);
    if (key == 27 || key == 'q') break;
    if (key == ' ') cv::waitKey(0);  // pause
  }
  return 0;
}
