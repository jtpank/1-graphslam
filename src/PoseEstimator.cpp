#include "PoseEstimator.hpp"

PoseEstimator::PoseEstimator(std::ofstream &file_out) : m_pOrb(cv::ORB::create()),
m_bfMatcher(cv::BFMatcher::create(cv::NORM_HAMMING)),
m_file_out(file_out)
{
    std::cout << "Constructing PoseEstimator" << std::endl;
    m_first_frame = false;
}

void PoseEstimator::set_first_frame()
{
  if(!m_first_frame)
    m_first_frame = true;
}

bool PoseEstimator::found_first_frame()
{
  return m_first_frame;
}

void PoseEstimator::insert_frame(int id, std::vector<cv::KeyPoint> kps, cv::Mat des)
{
  m_frames.emplace_back(id, std::move(kps), std::move(des));
  if (m_frames.size() > 2) m_frames.pop_front();
}

void PoseEstimator::orbDetectAndCompute(cv::Mat &inputFrame, cv::Mat &des, std::vector<cv::KeyPoint> &kps)
{

  m_pOrb->detect(inputFrame, kps);
  m_pOrb->compute(inputFrame, kps, des);
}

void PoseEstimator::match_frames(float ratio, std::vector<cv::DMatch> &good_matches)
{
  if(m_frames.size() > 2)
  {
    std::cerr << "frames size exceeds 2. how did this happen?" << std::endl;
  }
  if(m_frames[0].descriptors().empty() || m_frames[1].descriptors().empty())
  {
    //std::cout << "Descriptors are empty in one of the frames" << std::endl;
    return;
  }
  std::vector<std::vector<cv::DMatch>> knn;
  m_bfMatcher->knnMatch(m_frames[0].descriptors(), m_frames[1].descriptors(), knn, 2);

  good_matches.reserve(knn.size());
  for (const auto& m : knn)
  {
    if (m.size() < 2) continue;
    if (m[0].distance < ratio * m[1].distance)
      good_matches.push_back(m[0]);
  }
}

void PoseEstimator::matches_to_points(
    const std::vector<cv::DMatch>& matches,
    std::vector<cv::Point2f>& pts0,
    std::vector<cv::Point2f>& pts1)
{
  pts0.clear(); pts1.clear();
  pts0.reserve(matches.size());
  pts1.reserve(matches.size());

  auto kps0 = m_frames[0].keypoints();
  auto kps1 = m_frames[1].keypoints();

  for (const auto& m : matches)
  {
    pts0.push_back(kps0[m.queryIdx].pt);
    pts1.push_back(kps1[m.trainIdx].pt);
  }
}

void PoseEstimator::generate_pose(double focal_length, cv::Point2d &principal_point,
    std::vector<cv::Point2f>& pts0,
    std::vector<cv::Point2f>& pts1,
    double timestamp_ms)
{
  //principal point is the exact center in pix coords of the pixel plane
  //fundamental, essential matrix
  cv::Mat f_m, e_m;
  f_m = cv::findFundamentalMat(pts0, pts1, cv::FM_8POINT);
  std::cout << "fundamental_mat: " << std::endl << f_m << std::endl;

  e_m = cv::findEssentialMat(pts0, pts1, focal_length, principal_point);
  std::cout << "essential_mat: " << std::endl << e_m << std::endl;

  //Q: do we need homography matrix here using RANSAC?

  cv::Mat R, t;
  cv::recoverPose(e_m, pts0, pts1, R, t, focal_length, principal_point);

  Eigen::Matrix3d eigen_mat;
  cv::cv2eigen(R, eigen_mat);
  Eigen::Quaterniond quat(eigen_mat);
  quat.normalize();
  std::cout << "Quaternion (x, y, z, w): "
          << quat.x() << ", "
          << quat.y() << ", "
          << quat.z() << ", "
          << quat.w() << std::endl;
  std::cout << "R: " << std::endl << R << std::endl;
  std::cout << "t: " << std::endl << t << std::endl;

double tx = t.at<double>(0,0);
double ty = t.at<double>(1,0);
double tz = t.at<double>(2,0);
std::string line = std::format(
    "{:.4f} {:.4f} {:.4f} {:.4f} {:.6f} {:.6f} {:.6f} {:.6f}",
    timestamp_ms,
    tx, ty, tz,
    quat.x(), quat.y(), quat.z(), quat.w()
);
  write_line(line);
}

//file io helper
void PoseEstimator::write_line(std::string &line)
{
  if(!m_file_out.is_open())
  {
    std::cerr << "file is not opened!" << std::endl;
    return;
  }
  m_file_out << line << "\n";
}
