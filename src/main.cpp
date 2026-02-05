#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <memory>
#include <thread>
#include <cstdio>
#include <chrono>
#include <string>
#include <vector>
#include "PoseEstimator.hpp"

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
  std::string orb_window_name = "orb_kps";
  if (argc < 2)
  {
    std::cerr << "Usage: " << argv[0] << " video.mp4" << std::endl;
    return -1;
  }
  std::string video_path = argv[1];

  cv::VideoCapture cap(argv[1], cv::CAP_FFMPEG);
  if(!cap.isOpened())
  {
    std::cerr << "Error no video capture!" << std::endl;
    return -1;
  }
  cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(orb_window_name, cv::WINDOW_AUTOSIZE);

  cv::Mat p_frame, c_frame, gray, vis, orb_vis, prev_vis;
  std::unique_ptr<PoseEstimator> p_pe = std::make_unique<PoseEstimator>();
  int id = 0;

  for(;;)
  {
    id++;
    cap >> c_frame;
    if(c_frame.empty())
    {
      std::cerr << "cannot read frame from stream" << std::endl;
      break;
    }
    cv::cvtColor(c_frame, gray, cv::COLOR_BGR2GRAY);
    std::vector<cv::Point2f> corners;
    extract_features(corners, gray);

    cv::Mat des;
    std::vector<cv::KeyPoint> kps;

    p_pe->orbDetectAndCompute(gray, des, kps);
    orb_vis = c_frame.clone();
    for(const auto& kp: kps)
    {
      cv::Point int_pt = cv::Point(static_cast<int>(kp.pt.x), static_cast<int>(kp.pt.y));
      cv::circle(orb_vis, int_pt, 3, cv::Scalar(0,255,0), -1);
    }

    vis = c_frame.clone();
    for (const auto& pt : corners)
    {
      cv::circle(vis, pt, 3, cv::Scalar(0, 255, 0), -1);
    }
 

    p_pe->insert_frame(id, std::move(kps), std::move(des));
    if(!p_pe->found_first_frame())
    {
      p_pe->set_first_frame();
      continue;
    }
    //otherwise we can compute matches
    float ratio = 0.7;
    std::vector<cv::DMatch> good_matches;
    p_pe->match_frames(ratio, good_matches);
    std::vector<cv::Point2f> pts0, pts1;
    p_pe->matches_to_points(good_matches, pts0, pts1);


    cv::imshow(orb_window_name, orb_vis);
    cv::imshow(window_name, vis);
    if(cv::waitKey(1) == 'q')
      break;

  }

  cap.release();
  cv::destroyAllWindows();

  return 0;
}
