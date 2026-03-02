#include "Frames.hpp"
Frames::Frames(int id) : m_frame_id(id)
{
}

Frames::Frames(int id, std::vector<cv::KeyPoint>&& kps, cv::Mat&& des, cv::Mat&& input_frame) noexcept : m_frame_id(id),
  m_kps(std::move(kps)), m_des(std::move(des)), m_frame(std::move(input_frame))
{

}

int Frames::get_frame_id()
{
  return m_frame_id;
}

cv::Mat Frames::descriptors()
{
  return m_des;
}
std::vector<cv::KeyPoint> Frames::keypoints()
{
  return m_kps;
}

cv::Mat Frames::frame()
{
  return m_frame;
}
