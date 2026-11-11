#include "trajectory_eval.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>

#include <Eigen/Dense>
#include <glog/logging.h>

namespace svo {

bool writeTumTrajectory(const std::string& filename, const StampedPoses& poses)
{
  std::ofstream f(filename);
  if (!f.is_open())
  {
    LOG(ERROR) << "Cannot write " << filename;
    return false;
  }
  f << "# timestamp tx ty tz qx qy qz qw\n";
  f << std::fixed;
  for (const auto& p : poses)
  {
    f << std::setprecision(9) << static_cast<double>(p.timestamp_ns) * 1e-9 << " "
      << std::setprecision(9) << p.p.x() << " " << p.p.y() << " " << p.p.z() << " "
      << p.q.x() << " " << p.q.y() << " " << p.q.z() << " " << p.q.w() << "\n";
  }
  return true;
}

bool writeEurocTrajectory(const std::string& filename, const StampedPoses& poses)
{
  std::ofstream f(filename);
  if (!f.is_open())
  {
    LOG(ERROR) << "Cannot write " << filename;
    return false;
  }
  f << "#timestamp [ns],p_RS_R_x [m],p_RS_R_y [m],p_RS_R_z [m],q_RS_w [],"
       "q_RS_x [],q_RS_y [],q_RS_z []\n";
  f << std::fixed << std::setprecision(9);
  for (const auto& p : poses)
  {
    f << p.timestamp_ns << "," << p.p.x() << "," << p.p.y() << "," << p.p.z()
      << "," << p.q.w() << "," << p.q.x() << "," << p.q.y() << "," << p.q.z()
      << "\n";
  }
  return true;
}

std::string alignmentName(AlignmentType type)
{
  switch (type)
  {
    case AlignmentType::kPosYaw: return "posyaw";
    case AlignmentType::kSE3: return "se3";
    case AlignmentType::kSim3: return "sim3";
    case AlignmentType::kNone: return "none";
  }
  return "?";
}

namespace {

bool interpolateGt(const StampedPoses& gt, int64_t ts, PoseEntry* out)
{
  if (gt.empty() || ts < gt.front().timestamp_ns || ts > gt.back().timestamp_ns)
    return false;
  auto it = std::lower_bound(
      gt.begin(), gt.end(), ts,
      [](const PoseEntry& a, int64_t t) { return a.timestamp_ns < t; });
  if (it == gt.end())
    return false;
  if (it->timestamp_ns == ts || it == gt.begin())
  {
    *out = *it;
    out->timestamp_ns = ts;
    return true;
  }
  auto prev = it - 1;
  const double dt = static_cast<double>(it->timestamp_ns - prev->timestamp_ns);
  if (dt > 0.5e9)  // gap in groundtruth larger than 0.5 s (GPS: ~10 Hz)
    return false;
  const double a = static_cast<double>(ts - prev->timestamp_ns) / dt;
  out->timestamp_ns = ts;
  out->p = (1.0 - a) * prev->p + a * it->p;
  out->q = prev->q.slerp(a, it->q);
  return true;
}

} // namespace

AteResult computeAte(const StampedPoses& estimate, const StampedPoses& gt,
                     AlignmentType type, StampedPoses* aligned_estimate,
                     StampedPoses* matched_gt)
{
  AteResult res;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> E, G;
  StampedPoses est_matched, gt_matched;
  for (const auto& e : estimate)
  {
    PoseEntry g;
    if (interpolateGt(gt, e.timestamp_ns, &g))
    {
      E.push_back(e.p);
      G.push_back(g.p);
      est_matched.push_back(e);
      gt_matched.push_back(g);
    }
  }
  res.num_matches = E.size();
  if (E.size() < 10)
    return res;

  const size_t n = E.size();
  Eigen::Vector3d mu_e = Eigen::Vector3d::Zero(), mu_g = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < n; ++i)
  {
    mu_e += E[i];
    mu_g += G[i];
  }
  mu_e /= static_cast<double>(n);
  mu_g /= static_cast<double>(n);

  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  double s = 1.0;
  if (type == AlignmentType::kNone)
  {
    R.setIdentity();
    mu_e.setZero();
    mu_g.setZero();
  }
  else if (type == AlignmentType::kPosYaw)
  {
    double a = 0.0, b = 0.0;
    for (size_t i = 0; i < n; ++i)
    {
      const Eigen::Vector3d e = E[i] - mu_e;
      const Eigen::Vector3d g = G[i] - mu_g;
      a += g.x() * e.x() + g.y() * e.y();
      b += g.y() * e.x() - g.x() * e.y();
    }
    const double yaw = std::atan2(b, a);
    R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  }
  else
  {
    // Umeyama
    Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
    double var_e = 0.0;
    for (size_t i = 0; i < n; ++i)
    {
      const Eigen::Vector3d e = E[i] - mu_e;
      const Eigen::Vector3d g = G[i] - mu_g;
      C += g * e.transpose();
      var_e += e.squaredNorm();
    }
    C /= static_cast<double>(n);
    var_e /= static_cast<double>(n);
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(C, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
    if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0)
      S(2, 2) = -1;
    R = svd.matrixU() * S * svd.matrixV().transpose();
    if (type == AlignmentType::kSim3 && var_e > 1e-12)
      s = (svd.singularValues().asDiagonal() * S).trace() / var_e;
  }
  const Eigen::Vector3d t = mu_g - s * R * mu_e;

  std::vector<double> err(n);
  double sum2 = 0.0, sum = 0.0;
  for (size_t i = 0; i < n; ++i)
  {
    const Eigen::Vector3d ea = s * R * E[i] + t;
    err[i] = (ea - G[i]).norm();
    sum2 += err[i] * err[i];
    sum += err[i];
  }
  for (size_t i = 1; i < n; ++i)
    res.trajectory_length += (G[i] - G[i - 1]).norm();

  res.valid = true;
  res.rmse = std::sqrt(sum2 / static_cast<double>(n));
  res.mean = sum / static_cast<double>(n);
  std::vector<double> sorted = err;
  std::sort(sorted.begin(), sorted.end());
  res.median = sorted[n / 2];
  res.max = sorted.back();
  res.scale = s;
  res.R = R;
  res.t = t;

  if (aligned_estimate)
  {
    aligned_estimate->clear();
    const Eigen::Quaterniond qR(R);
    for (const auto& e : est_matched)
    {
      PoseEntry a = e;
      a.p = s * R * e.p + t;
      a.q = qR * e.q;
      aligned_estimate->push_back(a);
    }
  }
  if (matched_gt)
    *matched_gt = gt_matched;
  return res;
}

} // namespace svo
