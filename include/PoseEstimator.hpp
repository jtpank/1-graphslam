#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/types.hpp>
#include <vector>
#include <cstdio>
#include <iostream>

class PoseEstimator {
public:
    PoseEstimator();
    ~PoseEstimator();
    void frameMatcher();
    void orbDetectAndCompute(cv::Mat &inputFrame, cv::Mat &outputFrame); 

private:
    cv::Ptr<cv::ORB> m_pOrb;
    cv::Ptr<cv::BFMatcher> m_bfMatcher;
    

};
