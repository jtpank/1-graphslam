#include "PoseEstimator.hpp"

PoseEstimator::PoseEstimator() : m_pOrb(cv::ORB::create()),
m_bfMatcher(cv::BFMatcher::create(cv::NORM_HAMMING))
{
    std::cout << "Constructing PoseEstimator" << std::endl;
}


void PoseEstimator::orbDetectAndCompute(cv::Mat &inputFrame, cv::Mat &outputFrame)
{
    cv::Mat des;


}

void PoseEstimator::frameMatcher()
{
    return;
}
