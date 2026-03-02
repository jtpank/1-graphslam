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

#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <unistd.h>

//custom includes
#include "PoseEstimator.hpp"
#include "camera_parameters.hpp"
using namespace std;
using namespace Eigen;
void draw_keypoints_on_image(std::vector<cv::KeyPoint> &kps, cv::Mat &frame)
{
    for(const auto& kp: kps)
    {
      cv::Point int_pt = cv::Point(static_cast<int>(kp.pt.x), static_cast<int>(kp.pt.y));
      cv::circle(frame, int_pt, 3, cv::Scalar(0,255,0), -1);
    }
}

void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses) {
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
    pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glLineWidth(2);
        for (size_t i = 0; i < poses.size(); i++) {
            Vector3d Ow = poses[i].translation();
            Vector3d Xw = poses[i] * (0.1 * Vector3d(1, 0, 0));
            Vector3d Yw = poses[i] * (0.1 * Vector3d(0, 1, 0));
            Vector3d Zw = poses[i] * (0.1 * Vector3d(0, 0, 1));
            glBegin(GL_LINES);
            glColor3f(1.0, 0.0, 0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Xw[0], Xw[1], Xw[2]);
            glColor3f(0.0, 1.0, 0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Yw[0], Yw[1], Yw[2]);
            glColor3f(0.0, 0.0, 1.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Zw[0], Zw[1], Zw[2]);
            glEnd();
        }
        for (size_t i = 0; i < poses.size(); i++) {
            glColor3f(0.0, 0.0, 0.0);
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i+1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000); // sleep 5 ms
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
  std::string orb_window_name = "orb_kps";
  std::string side_by_side_window = "side_by_side";
  if (argc != 3)
  {
      std::cerr << "Usage:\n"
                << "  ./slam --plot trajectory.txt\n"
                << "  ./slam --rec video.mp4\n";
      return 1;
  }

  std::string flag  = argv[1];
  std::string input = argv[2];

  if (flag == "--plot")
  {
      std::cout << "Plot mode\n";
      std::cout << "Trajectory file: " << input << "\n";
      std::string trajectory_file = input;
      vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses;
      ifstream fin(trajectory_file);
      if (!fin) {
          cout << "cannot find trajectory file at " << trajectory_file << endl;
          return 1;
      }

      while (!fin.eof()) {
          double time, tx, ty, tz, qx, qy, qz, qw;
          fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
          Isometry3d Twr(Quaterniond(qw, qx, qy, qz));
          Twr.pretranslate(Vector3d(tx, ty, tz));
          poses.push_back(Twr);
      }
      cout << "read total" << poses.size() << "pose entries" << endl;

      // draw trajectory in pangolin
      DrawTrajectory(poses);

  }
  else if (flag == "--vid")
  {
    std::cout << "Recording mode\n";
    std::cout << "Video file: " << input << "\n";
    std::string video_path = input;

    cv::VideoCapture cap(argv[2], cv::CAP_FFMPEG);
    if(!cap.isOpened())
    {
      std::cerr << "Error no video capture!" << std::endl;
      return -1;
    }
    cv::namedWindow(orb_window_name, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(side_by_side_window, cv::WINDOW_AUTOSIZE);


    //Frame processing
    double fx = Cameras::RealSense_D435i::fx;
    double fy = Cameras::RealSense_D435i::fy;
    double cx = Cameras::RealSense_D435i::cx;
    double cy = Cameras::RealSense_D435i::cy;
    cv::Mat K_mat = (cv::Mat_<double>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    cv::Mat p_frame, c_frame, gray, orb_vis, prev_vis;
    std::ofstream file_out;
    std::string output_file_name = "output_data.txt";
    file_out.open(output_file_name, std::ios::out | std::ios::trunc);
    std::unique_ptr<PoseEstimator> p_pe = std::make_unique<PoseEstimator>(file_out);
    int id = 0;
    for(;;)
    {
      // std::cout << std::format("id: {}\n", id);
      id++;
      cap >> c_frame;
      int height = c_frame.rows;
      int width = c_frame.cols;
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

      p_pe->insert_frame(id, std::move(kps), std::move(des), orb_vis.clone());
      if(!p_pe->found_first_frame())
      {
        p_pe->set_first_frame();
        continue;
      }
      //otherwise we can compute matches and generate pose
      float ratio = 0.7;
      std::vector<cv::DMatch> good_matches;
      std::vector<cv::Point2f> pts0, pts1;

      p_pe->match_frames(ratio, good_matches);
      p_pe->matches_to_points(good_matches, pts0, pts1);
      double timestamp_ms = cap.get(cv::CAP_PROP_POS_MSEC);
      std::vector<cv::Point3d> points_3d;
      if(pts0.size() > 8 && pts1.size() > 8)
      {
        p_pe->generate_pose(K_mat, pts0, pts1, good_matches, timestamp_ms);
      } 

      std::deque<Frames> deq_frames = p_pe->get_frames();
      //Assume they are the same size!
      cv::Mat concat_frames;
      cv::hconcat(deq_frames.back().frame(), deq_frames.front().frame(), concat_frames);
      //draw lines
      for(int i = 0; i < pts0.size(); ++i)
      {
        cv::Point2f point1 = pts1[i] + cv::Point2f(static_cast<float>(width), 0.0f);
        cv::line(concat_frames, pts0[i], point1, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
      }
      cv::imshow(side_by_side_window, concat_frames);
      

      cv::imshow(orb_window_name, orb_vis);
      if(cv::waitKey(1) == 'q')
        break;

    }
    file_out.close();
    std::cout << std::format("closing file {}\n", output_file_name);

    cap.release();
    cv::destroyAllWindows();


  }
  else
  {
      std::cerr << "Unknown option: " << flag << "\n";
      return 1;
  }
  
  return 0;
}
