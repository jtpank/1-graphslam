#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <memory>
#include <thread>
#include <cstdio>
#include <chrono>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <format>

//custom includes
#include "PoseEstimator.hpp"
#include "camera_parameters.hpp"

void draw_keypoints_on_image(std::vector<cv::KeyPoint> &kps, cv::Mat &frame)
{
    for(const auto& kp: kps)
    {
      cv::Point int_pt = cv::Point(static_cast<int>(kp.pt.x), static_cast<int>(kp.pt.y));
      cv::circle(frame, int_pt, 3, cv::Scalar(0,255,0), -1);
    }
}

int main(int argc, char** argv)
{
  /**cv::VideoCapture cap(2, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y','U','Y','V'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_CONVERT_RGB, false);
  **/

  //Window setup for testing
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


  //Frame processing

  cv::Mat p_frame, c_frame, gray, orb_vis, prev_vis;
  std::ofstream file_out;
  std::string output_file_name = "output_data.txt";
  file_out.open(output_file_name, std::ios::out | std::ios::trunc);
  std::unique_ptr<PoseEstimator> p_pe = std::make_unique<PoseEstimator>(file_out);
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
    cv::Mat des;
    std::vector<cv::KeyPoint> kps;

    p_pe->orbDetectAndCompute(gray, des, kps);
    orb_vis = c_frame.clone();

    draw_keypoints_on_image(kps, orb_vis);

    p_pe->insert_frame(id, std::move(kps), std::move(des));
    if(!p_pe->found_first_frame())
    {
      p_pe->set_first_frame();
      continue;
    }
    //otherwise we can compute matches and generate pose
    float ratio = 0.7;
    std::vector<cv::DMatch> good_matches;
    std::vector<cv::Point2f> pts0, pts1;
    double focal_length = Cameras::RealSense_D435i::fx;
    cv::Point2d principal_point(Cameras::RealSense_D435i::cx, Cameras::RealSense_D435i::cy);
    p_pe->match_frames(ratio, good_matches);
    p_pe->matches_to_points(good_matches, pts0, pts1);
    double timestamp_ms = cap.get(cv::CAP_PROP_POS_MSEC);
    if(pts0.size() > 8 && pts1.size() > 8)
    {
      p_pe->generate_pose(focal_length, principal_point, pts0, pts1, timestamp_ms);
    }


    cv::imshow(orb_window_name, orb_vis);
    if(cv::waitKey(1) == 'q')
      break;

  }
  file_out.close();
  std::cout << std::format("closing file {}", output_file_name);

  cap.release();
  cv::destroyAllWindows();

  return 0;
}
