#include "vision.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <utils.hpp>
#include <pose.hpp>
#include <iostream>

namespace reva {

  VisionProcessor::VisionProcessor(std::string videopath, std::string camPath){
    this->cap = cv::VideoCapture(videopath);
    if (!cap.isOpened()) {
      std::cerr << "Le fichier :" << videopath << " n'est pas lisible ou n'existe pas." << std::endl;
      exit(1);
    }
  }
  VisionProcessor::VisionProcessor(int webcamid, std::string camPath){
    this->cap = cv::VideoCapture(webcamid);
    if (!cap.isOpened()) {
      std::cerr << "Impossible de recuperer le flux de la webcam :" << webcamid << std::endl;
      exit(1);
    }loadCameraCalibration(std::string camPath);
  }

  void VisionProcessor::loadCameraCalibration(std::string camPath){
    this->fps = this->cap.get(cv::CAP_PROP_FPS);
    this->delay = this->fps > 0 ? (int)(1000.0 / this->fps) : 30;

    cv::Mat frame;
    this->cap.read(frame);
    if (frame.rows > LARGECONST || frame.cols > LARGECONST)
      cv::resize(frame, frame, cv::Size(), REDUCTIONFACTOR, REDUCTIONFACTOR);

    cv::Size frameSize = frame.size();
    if (!loadCameraParams(camPath, this->cam, frameSize)){
      std::cerr << "Impossible de charger les parametres de la camera :" << camPath <<std::endl;
      exit(1);
    };
    std::cout << "K =\n" << this->cam.K << "\ndist = " << this->cam.dist << std::endl;
    std::cout << "Video Params : \n "
      << "width : " << frame.cols << "\n "
      << "hight : " << frame.rows << "\n "
      << std::endl;
  }

  cv::Mat getPaperMask(cv::Mat frame,cv::Point seed,int saturation,int step){
    return growRegion(frame, seed, saturation, step);
  }

  std::vector<Contours> getPaperContours(cv::Mat paperMask){
    std::vector<Contours> contours;    
    cv::findContours(paperMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    return contours;
  }

  Contours getPaperQuad(std::vector<Contours> contours){
    if (!contours.empty()) {
      auto biggest = *std::max_element(contours.begin(), contours.end(),
          [](auto& a, auto& b){ return cv::contourArea(a) < cv::contourArea(b); });
      std::vector<cv::Point> hull, quad;
      cv::convexHull(biggest, hull);
      double peri = cv::arcLength(hull, true);
      for (double eps = 0.02; eps <= 0.06 && quad.size() != 4; eps += 0.01)
        cv::approxPolyDP(hull, quad, eps * peri, true);
      return quad;
    }else{
      std::cerr << "getPaperQuad : Contours vides" << std::endl;
      exit(1);
    }
  }

}
