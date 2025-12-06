#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <memory>
#include <thread>
#include <cstdio>
#include <chrono>
#include <string>
#include <vector>

void extract_features(std::vector<cv::Point2f> &corners, cv::Mat &frame)
{
  int maxCorners = 100;
  double qualityLevel = 0.01;
  double minDistance = 10;
  cv::goodFeaturesToTrack(frame, corners, maxCorners, qualityLevel, minDistance);
}

int main(int argc, char** argv)
{

  cv::VideoCapture cap(0);
  std::string window_name = "camera_feed";
  if(!cap.isOpened())
  {
    std::cerr << "Error no video capture!" << std::endl;
    return -1;
  }
  cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
  cv::Mat frame;
  for(;;)
  {
    cap >> frame;
    if(frame.empty())
    {
      std::cerr << "cannot read frame from stream" << std::endl;
      break;
    }
    std::vector<cv::Point2f> corners;
    extract_features(corners, frame);
    cv::imshow(window_name, frame);
    if(cv::waitKey(1) == 'q')
      break;

  }

  cap.release();
  cv::destroyAllWindows();

  return 0;
}
