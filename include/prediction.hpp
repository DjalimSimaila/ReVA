#ifndef POINT_PREDICTOR
#define POINT_PREDICTOR
#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>

class PointPredictor{
  public:
    PointPredictor();
    void correct(cv::Mat measurement);
    cv::Point2f predict();
  private:
    cv::KalmanFilter* KF = new cv::KalmanFilter(4, 2, 0, CV_32F);
};

#endif // !POINT_PREDICTOR
