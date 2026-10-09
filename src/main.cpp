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

// Variables globales pour les trackbars
int main(int argc, char* argv[]) {
  cv::VideoCapture cap("../b.mp4");
  if ( !cap.isOpened() )  
  {
    std::cerr << "il faut ../b.mp4 le sang" << std::endl;
    exit(1);
  }


  double fps = cap.get(cv::CAP_PROP_FPS); //get the frames per seconds of the video
  namedWindow("A_good_name",cv::WINDOW_AUTOSIZE); //create a window called "MyVideo"

  while(true)
  {
    cv::Mat frame;
    if (!cap.read(frame)) {
      cap.set(cv::CAP_PROP_POS_FRAMES, 0);
      cap.read(frame);
    }
    std::vector<cv::Rect> rects;
    cv::Mat roi;
    cv::Rect rect = findPaperSheet(frame,rects,roi);
    rectangle(frame, rect, CV_RGB(0, 255, 0));
    cv::Mat black = cv::Mat::zeros(frame.size(),roi.type());
    roi.copyTo(black(rect));

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
      if (quad.size() == 4)
          for (auto& p : quad) cv::circle(frame, p, 6, cv::Scalar(0,0,255), cv::FILLED);
      }

    imshow("mask",black);
    imshow("A_good_name", frame);
    cv::waitKey(60000);
  }
  return 0;
}
