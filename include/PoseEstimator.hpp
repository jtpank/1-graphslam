#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/types.hpp>
#include <vector>
#include <deque>
#include <cstdio>
#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "Frames.hpp"
#include <fstream>
#include <format>

class PoseEstimator {
  public:
    PoseEstimator(std::ofstream &file_out);
    void frameMatcher();
    void orbDetectAndCompute(cv::Mat &inputFrame, cv::Mat &des, std::vector<cv::KeyPoint> &kps);
    void set_first_frame();
    bool found_first_frame();
    void insert_frame(int id, std::vector<cv::KeyPoint> kps, cv::Mat des, cv::Mat input_frame);
    void match_frames(float ratio, std::vector<cv::DMatch> &good_matches);
    void matches_to_points(
        const std::vector<cv::DMatch>& matches,
        std::vector<cv::Point2f>& pts0,
        std::vector<cv::Point2f>& pts1);

    void generate_pose(cv::Mat& K_mat,
        std::vector<cv::Point2f>& pts0,
        std::vector<cv::Point2f>& pts1,
        const std::vector<cv::DMatch>& good_matches,
        double timestamp_ms);
    void triangulation(
      const std::vector<cv::KeyPoint> &kp1,
      const std::vector<cv::KeyPoint> &kp2,
      const std::vector<cv::DMatch> &matches,
      const cv::Mat &R, const cv::Mat &t, const cv::Mat &K,
      std::vector<cv::Point3d> &points_3d
    );
    cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K);
    void write_line(std::string &line);

    const std::deque<Frames>& get_frames() const {
      return m_frames;
    }

  private:
    std::deque<Frames> m_frames;
    cv::Ptr<cv::ORB> m_pOrb;
    cv::Ptr<cv::BFMatcher> m_bfMatcher;
    bool m_first_frame = false;
    std::ofstream &m_file_out;
    Eigen::Quaterniond m_q_w_c = Eigen::Quaterniond::Identity(); // world->current orientation
    Eigen::Vector3d m_t_w_c = Eigen::Vector3d::Zero();          // world position of current camera

};
