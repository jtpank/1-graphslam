#include "PoseEstimator.hpp"

PoseEstimator::PoseEstimator(std::ofstream &file_out) : m_pOrb(cv::ORB::create(
        2000,          // nfeatures (default 500 → too low for VO)
        1.2f,          // scaleFactor
        8,             // nlevels
        31,            // edgeThreshold
        0,             // firstLevel
        2,             // WTA_K
        cv::ORB::HARRIS_SCORE, // better for SLAM than FAST_SCORE
        31,            // patchSize
        15             // fastThreshold (lower → more features)
)),
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

void PoseEstimator::insert_frame(int id, std::vector<cv::KeyPoint> kps, cv::Mat des, cv::Mat input_frame)
{
  m_frames.emplace_back(id, std::move(kps), std::move(des), std::move(input_frame));
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
  good_matches.clear();
  const int max_hamming = 50; // try 40-60 for ORB
  for (const auto& m : knn)
  {
    if (m.size() < 2) continue;
    if (m[0].distance < ratio * m[1].distance && m[0].distance <= max_hamming)
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

cv::Point2d PoseEstimator::pixel2cam(const cv::Point2d &p, const cv::Mat &K) {
  return cv::Point2d
    (
      (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
      (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}

void PoseEstimator::generate_pose(cv::Mat& K_mat,
    std::vector<cv::Point2f>& pts0,
    std::vector<cv::Point2f>& pts1,
    const std::vector<cv::DMatch>& good_matches,
    double timestamp_ms)
{
  double focal_length = K_mat.at<double>(0,0);
  cv::Point2d principal_point(K_mat.at<double>(0,2), K_mat.at<double>(1,2));
  //principal point is the exact center in pix coords of the pixel plane
  //fundamental, essential matrix
  cv::Mat f_m, e_m;
  // f_m = cv::findFundamentalMat(pts0, pts1, cv::FM_8POINT);
  // std::cout << "fundamental_mat: " << std::endl << f_m << std::endl;

  cv::Mat mask;
  e_m = cv::findEssentialMat(
      pts0, pts1,
      focal_length, principal_point,
      cv::RANSAC,
      0.999,      // prob
      1.0,        // threshold in pixels (tune: 0.5-2.0)
      mask
  );

  // std::cout << "essential_mat: " << std::endl << e_m << std::endl;

  //Q: do we need homography matrix here using RANSAC?

  cv::Mat R, t;
  cv::recoverPose(e_m, pts0, pts1, R, t, focal_length, principal_point);

  cv::Mat t_x =
    (cv::Mat_<double>(3, 3) << 0, -t.at<double>(2, 0), t.at<double>(1, 0),
      t.at<double>(2, 0), 0, -t.at<double>(0, 0),
      -t.at<double>(1, 0), t.at<double>(0, 0), 0);

  std::cout << "t^R=" << std::endl << t_x * R << std::endl;
  for (cv::DMatch m: good_matches) {
      auto kps0 = m_frames[0].keypoints();
      auto kps1 = m_frames[1].keypoints();
      cv::Point2d pt1 = pixel2cam(kps0[m.queryIdx].pt, K_mat);
      cv::Mat y1 = (cv::Mat_<double>(3, 1) << pt1.x, pt1.y, 1);
      cv::Point2d pt2 = pixel2cam(kps1[m.trainIdx].pt, K_mat);
      cv::Mat y2 = (cv::Mat_<double>(3, 1) << pt2.x, pt2.y, 1);
      cv::Mat d = y2.t() * t_x * R * y1;
      // std::cout << "epipolar constraint = " << d << std::endl;
  }
  Eigen::Matrix3d Rrel;
  cv::cv2eigen(R, Rrel);
  Eigen::Quaterniond q_rel(Rrel);
  q_rel.normalize();

  Eigen::Vector3d t_rel(
    t.at<double>(0,0),
    t.at<double>(1,0),
    t.at<double>(2,0)
  );
  Eigen::Vector3d t_dir = t_rel.normalized();
  double step_size = 0.05; //meters? aribtrary
  static Eigen::Vector3d prev_dir = Eigen::Vector3d::Zero();
  if (prev_dir.squaredNorm() > 0.0 && t_dir.dot(prev_dir) < 0.0)
    t_dir = -t_dir;

  // angular gating -- what is this doing
  double cosang = (prev_dir.squaredNorm() > 0.0)
                      ? t_dir.dot(prev_dir)
                      : 1.0;

  // If direction jumps too much, skip translation this frame
  if (cosang < 0.7) {   // ~45 degrees threshold
      std::cout << "Skipping translation update (degenerate frame)\n";
      t_dir = prev_dir;          // or: just set t_step = 0 below
  }

  prev_dir = t_dir;

  Eigen::Vector3d t_step = step_size * t_dir;
  // state: world-from-camera
  Eigen::Quaterniond q_w_c = m_q_w_c;
  Eigen::Vector3d    t_w_c = m_t_w_c;

  // relative: cam1->cam2 in cam1 coords
  Eigen::Quaterniond q_c1_c2 = q_rel;
  Eigen::Vector3d    t_c1_c2 = t_step;

  // update
  Eigen::Quaterniond q_w_c_new = (q_w_c * q_c1_c2).normalized();
  Eigen::Vector3d    t_w_c_new = t_w_c + (q_w_c * t_c1_c2);

  // sign continuity for plotting
  if (q_w_c.dot(q_w_c_new) < 0.0) q_w_c_new.coeffs() *= -1.0;

  m_q_w_c = q_w_c_new;
  m_t_w_c = t_w_c_new;

  std::string line = std::format(
      "{:.3f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}",
      timestamp_ms,
      m_t_w_c.x(), m_t_w_c.y(), m_t_w_c.z(),
      m_q_w_c.x(), m_q_w_c.y(), m_q_w_c.z(), m_q_w_c.w()
  );
  write_line(line);
}


void PoseEstimator::triangulation(
  const std::vector<cv::KeyPoint> &kp1,
  const std::vector<cv::KeyPoint> &kp2,
  const std::vector<cv::DMatch> &matches,
  const cv::Mat &R, const cv::Mat &t, const cv::Mat &K_mat,
  std::vector<cv::Point3d> &points_3d
)
{
  cv::Mat T1 = (
    cv::Mat_<float>(3,4) <<
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0
    );
  
  cv::Mat T2 = (
    cv::Mat_<float>(3,4) <<
    R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
    R.at<double>(1,0), R.at<double>(0,1), R.at<double>(1,2), t.at<double>(1,0),
    R.at<double>(2,0), R.at<double>(0,1), R.at<double>(2,2), t.at<double>(2,0)
  );
  //convert pixel coordinates into camera coordinates
  std::vector<cv::Point2f> cam_pts0, cam_pts1;
  for (cv::DMatch m: matches) {
      auto kps0 = m_frames[0].keypoints();
      auto kps1 = m_frames[1].keypoints();
      cam_pts0.push_back(pixel2cam(kps0[m.queryIdx].pt, K_mat));
      cam_pts1.push_back(pixel2cam(kps1[m.trainIdx].pt, K_mat));
  }

  cv::Mat pts_4d;
  cv::triangulatePoints(T1, T2, cam_pts0, cam_pts1, pts_4d);
  for (int i = 0; i < pts_4d.cols; i++) {
    cv::Mat x = pts_4d.col(i);
    x /= x.at<float>(3, 0); 
    cv::Point3d p(
      x.at<float>(0, 0),
      x.at<float>(1, 0),
      x.at<float>(2, 0)
    );
    points_3d.push_back(p);
  }
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
