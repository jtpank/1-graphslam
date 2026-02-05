#pragma once
#include <vector>
#include <utility>
#include <opencv2/core.hpp>

class Frames {
public:
  Frames(int id);
  Frames(int id, std::vector<cv::KeyPoint>&& kps,
         cv::Mat&& des) noexcept;

  int get_frame_id();
  cv::Mat descriptors();
  std::vector<cv::KeyPoint> keypoints();

private:
  int m_frame_id;
  std::vector<cv::KeyPoint> m_kps;
  cv::Mat m_des;

};
