#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/types.hpp>
#include <vector>
#include <deque>
#include <cstdio>
#include <iostream>
#include "Frames.hpp"

class PoseEstimator {
  public:
    PoseEstimator();
    void frameMatcher();
    void orbDetectAndCompute(cv::Mat &inputFrame, cv::Mat &des, std::vector<cv::KeyPoint> &kps);
    void set_first_frame();
    bool found_first_frame();
    void insert_frame(int id, std::vector<cv::KeyPoint> kps, cv::Mat des);
    void match_frames(float ratio, std::vector<cv::DMatch> &good_matches);
    void matches_to_points(
        const std::vector<cv::DMatch>& matches,
        std::vector<cv::Point2f>& pts0,
        std::vector<cv::Point2f>& pts1);

    void generate_pose(double focal_length, cv::Point2d &principal_point,
        std::vector<cv::Point2f>& pts0,
        std::vector<cv::Point2f>& pts1);

  private:
    std::deque<Frames> m_frames;
    cv::Ptr<cv::ORB> m_pOrb;
    cv::Ptr<cv::BFMatcher> m_bfMatcher;
    bool m_first_frame = false;

};
