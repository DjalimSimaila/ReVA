#include <cstdlib>
#include <opencv2/imgproc.hpp>
#include <queue>
#include <utils.hpp>


cv::Mat growRegion(const cv::Mat& image, cv::Point seed, int satTol, int step) {
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
      if (q.x < 0 || q.y < 0 || q.x >= img.cols || q.y >= img.rows) continue;  
      if (mask.at<uchar>(q)) continue;                                          

      const cv::Vec3b c = img.at<cv::Vec3b>(q);  
      bool peuColore   = std::abs(c[1] - ref[1]) <= satTol;  
      bool pasTropNoir = c[2] >= ref[2] / 2;                
      bool continu     = std::abs(c[1] - cp[1]) <= step &&  
                         std::abs(c[2] - cp[2]) <= step;  

      if (peuColore && pasTropNoir && continu) {
        mask.at<uchar>(q) = 255;
        todo.push(q);  
      }
    }
  }
  return mask;
}
