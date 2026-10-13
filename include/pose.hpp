#ifndef POSE_HPP
#define POSE_HPP

#include <opencv2/core.hpp>
#include <string>
#include <vector>

struct CameraParams {
  cv::Mat K;     // matrice intrinsèque 3x3
  cv::Mat dist;  // distorsion (k1, k2, p1, p2, k3)
};

// Charge K et dist depuis le YAML de calibration. Si frameSize est donné,
// K est mis à l'échelle de la résolution de la vidéo.
bool loadCameraParams(const std::string& path, CameraParams& cam,
                      cv::Size frameSize = cv::Size());

// Ordonne les 4 coins dans le même ordre que a4ObjectPoints.
std::vector<cv::Point2f> orderCorners(const std::vector<cv::Point>& quad);

// Coins de la feuille en mm, origine au centre, Z vers la caméra.
std::vector<cv::Point3f> a4ObjectPoints(float w = 297.f, float h = 210.f);

// Pose de la feuille par rapport à la caméra. reprojErr : erreur moyenne en px.
bool estimatePose(const std::vector<cv::Point2f>& imgPts, const CameraParams& cam,
                  cv::Mat& rvec, cv::Mat& tvec, double* reprojErr = nullptr);

// Carré vert posé sur la feuille.
void drawBase(cv::Mat& img, const CameraParams& cam, const cv::Mat& rvec,
              const cv::Mat& tvec, float side = 100.f);

#endif
