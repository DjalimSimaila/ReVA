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
    std::cerr << "Cannot open the video file." << std::endl;
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
    cv::Mat out = cv::Mat::zeros(frame.size(),roi.type());
    roi.copyTo(black(rect));
    std::vector<std::vector<cv::Point>> contours;
    cv::cornerHarris(black,out,2,3,0.04);
    cv::Mat dst_norm, dst_norm_scaled;
    cv::normalize( out, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
    cv::convertScaleAbs( dst_norm, dst_norm_scaled );
    for( int i = 0; i < dst_norm.rows ; i++ )
    {
        for( int j = 0; j < dst_norm.cols; j++ )
        {
            if( (int) dst_norm.at<float>(i,j) > 200 )
            {
                circle( dst_norm_scaled, cv::Point(j,i), 5,  cv::Scalar(0), 2, 8, 0 );
            }
        }
    }
    cv::findContours(black,contours,cv::RETR_CCOMP,cv::CHAIN_APPROX_SIMPLE);
    for (const std::vector obj : contours){
       for (const cv::Point& pt : obj)
         cv::circle(frame, pt, 2, cv::Scalar(0, 0, 255), cv::FILLED);
    }
    
    // cv::drawContours(frame, contours, -1, (0,255,0), 3);
    imshow("mask",black);
    imshow("coins",dst_norm_scaled);
    imshow("A_good_name", frame);
    cv::waitKey(60000);
  }
  return 0;
}
