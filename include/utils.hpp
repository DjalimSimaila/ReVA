#ifndef UTILS
#define UTILS

#include <opencv2/core.hpp>
#include <string>

cv::Mat loadimage(std::string filepath);
cv::Mat toGrayScale(const cv::Mat input);
void addLogoTo(const std::string imagePath, const std::string logo);
void blurRoi(const std::string imagePath, cv::Rect rect);
void checkerGrid(const std::string imagePath);
void reduceColor(const std::string imagePath, int factor);
cv::Mat pepperSalt(const std::string imagePath);
void sepiaFilter(const std::string imagePath);
void findColor(const std::string imagePath, const cv::Vec3b& couleurCible, double seuil);
void extractBackground(const std::string& inputImagePath);
void showHistogram(const std::string imagePath);
cv::Mat showHistogram(const cv::Mat image);
void chechNoise(const std::string imagePath);
cv::Rect findPaperSheet(cv::Mat image, std::vector<cv::Rect>& rects, cv::Mat& roi);

#endif
