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
  /**cv::VideoCapture cap(2, cv::CAP_V4L2);
  cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y','U','Y','V'));
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  cap.set(cv::CAP_PROP_CONVERT_RGB, false);

  **/
  std::string window_name = "camera_feed";
  if (argc < 2)
  {
    std::cerr << "Usage: " << argv[0] << " video.mp4" << std::endl;
    return -1;
  }
  std::string video_path = argv[1];

  cv::VideoCapture cap(argv[1], cv::CAP_FFMPEG);
  cv::Ptr<cv::ORB> m_pOrb;
  cv::Ptr<cv::BFMatcher> m_bfMatcher;
  if(!cap.isOpened())
  {
    std::cerr << "Error no video capture!" << std::endl;
    return -1;
  }
  cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
  cv::Mat p_frame, c_frame, gray, vis;
  bool first_frame_found = false;
  for(;;)
  {
    cap >> c_frame;
    if(c_frame.empty())
    {
      std::cerr << "cannot read frame from stream" << std::endl;
      break;
    }
    //cv::cvtColor(c_frame, gray, cv::COLOR_YUV2GRAY_YUY2);
    cv::cvtColor(c_frame, gray, cv::COLOR_BGR2GRAY);
    std::vector<cv::Point2f> corners;
    extract_features(corners, gray);

    //cv::cvtColor(c_frame, vis, cv::COLOR_YUV2BGR_YUY2);
    vis = c_frame.clone();
    for (const auto& pt : corners)
    {
      cv::circle(vis, pt, 3, cv::Scalar(0, 255, 0), -1);
    }
    cv::imshow(window_name, vis);
    if(cv::waitKey(1) == 'q')
      break;

  }

  cap.release();
  cv::destroyAllWindows();

  return 0;
}
