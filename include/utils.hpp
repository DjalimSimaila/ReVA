#ifndef UTILS
#define UTILS

#include <opencv2/core.hpp>

cv::Mat toGrayScale(const cv::Mat input);
cv::Rect findPaperSheet(cv::Mat image, std::vector<cv::Rect>& rects, cv::Mat& roi);
cv::Mat growRegion(const cv::Mat& image, cv::Point seed, int satTol, int step);

#endif
