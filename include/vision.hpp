#include <opencv2/core.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video/tracking.hpp>
#include <pose.hpp>

using Contours = std::vector<cv::Point>;

namespace reva {

  class VisionProcessor{

    private:
      cv::VideoCapture cap;
      cv::Point seed;
      CameraParams cam;
      double fps;
      int delay;

    public:
      VisionProcessor(std::string videopath = "../videos/prof.mp4",std::string camPath = "../input_camera.yaml");
      VisionProcessor(int webcamid = 0,std::string camPath = "../input_camera.yaml");
      void loadCameraCalibration();
      cv::Mat getFrame();
      cv::Mat getPaperMask(cv::Mat frame,cv::Point seed,int saturation,int step);
      std::vector<Contours> getPaperContours(cv::Mat paperMask);
      Contours getPaperQuad(std::vector<Contours> contours);

  };

}
