#include "prediction.hpp"
#include <opencv2/core/types.hpp>

PointPredictor::PointPredictor(){
  this->KF->transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0,
      0, 1, 0, 1,
      0, 0, 1, 0,
      0, 0, 0, 1);

  this->KF->measurementMatrix = (cv::Mat_<float>(2, 4) << 1, 0, 0, 0,
      0, 1, 0, 0);

  setIdentity(this->KF->processNoiseCov, cv::Scalar(1e-4));
  setIdentity(this->KF->measurementNoiseCov, cv::Scalar(1e-1));
  setIdentity(this->KF->errorCovPost, cv::Scalar(1));
  this->KF->statePost = (cv::Mat_<float>(4, 1) << 0, 0, 0, 0);
}

void PointPredictor::correct(cv::Mat measurement){
    this->KF->correct(measurement);
}

cv::Point2f PointPredictor::predict(){
    cv::Mat prediction = this->KF->predict();
    return cv::Point2f(prediction.at<float>(0), prediction.at<float>(1));
}
