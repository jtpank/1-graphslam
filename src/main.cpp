#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <memory>
#include <thread>
#include <cstdio>
#include <chrono>
#include <string>
#include <vector>
#include "Utilities.hpp"
#include "vehicle_dynamics.hpp"
#include "map_2d.hpp"

using namespace vehicle_dynamics;

void extract_features(std::vector<cv::Point2f> &corners, cv::Mat &frame)
{
  int maxCorners = 100;
  double qualityLevel = 0.01;
  double minDistance = 10;
  cv::goodFeaturesToTrack(frame, corners, maxCorners, qualityLevel, minDistance);
}

int main(int argc, char** argv)
{

  DriveParams myParams;
  myParams.track_width = 0.5;
  myParams.wheel_radius = 0.25;
  myParams.mass = 0.5;
  myParams.inertia = 1.0;
  myParams.max_velocity = 1.0;
  myParams.max_omega = 1.0;

  VehicleDynamics robot(myParams);

  VehicleState state;
  state.pose = Vector3d(0.0f, 0.0f, 0.0f);
  state.vel = Vector2d(0.0f, 0.0f);
  state.omega = 0.0f;
  state.a = 0.0f;
  state.alpha = 0.0f;

  ASCIIVisualizer viz(80, 40, 5.0f);

  std::vector<Vector2d> trail;

  float dt = 0.1f;
  int num_steps = 1000;

  Vector2d control_input;

  for (int i = 0; i < num_steps; i++) {
    if (i < 300) {
      control_input = Vector2d(0.5f, 0.5f);
    } else if (i < 600) {
      control_input = Vector2d(0.3f, 0.7f);
    } else {
      control_input = Vector2d(0.7f, 0.3f);
    }

    state = robot.integrate(state, control_input, dt);

    trail.push_back(Vector2d(state.pose(0), state.pose(1)));

    viz.clear();
    viz.drawTrail(trail);
    viz.drawRobot(state.pose(0), state.pose(1), state.pose(2));
    viz.render();

    std::cout << "Step " << i << ": "
                  << "x=" << state.pose(0) << " "
                  << "y=" << state.pose(1) << " "
                  << "theta=" << state.pose(2) << "\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::cout << "Simulation complete!\n";


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
