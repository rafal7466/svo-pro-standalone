#pragma once
// Trajectory output (TUM / EuRoC formats) and Absolute Trajectory Error
// evaluation against EuRoC groundtruth (replacement for rpg_trajectory_evaluation
// for a quick result directly after the run).

#include <string>
#include <vector>

#include "euroc_dataset.h"

namespace svo {

using StampedPoses = PoseEntries;

/// Write "timestamp[s] tx ty tz qx qy qz qw" (TUM format, also used by
/// rpg_trajectory_evaluation as stamped_traj_estimate.txt).
bool writeTumTrajectory(const std::string& filename, const StampedPoses& poses);

/// Write EuRoC style csv "#timestamp [ns],p_x,p_y,p_z,q_w,q_x,q_y,q_z".
bool writeEurocTrajectory(const std::string& filename, const StampedPoses& poses);

enum class AlignmentType
{
  kPosYaw,  ///< 4-DoF (translation + yaw): standard for VIO
  kSE3,     ///< 6-DoF rigid
  kSim3,    ///< 7-DoF similarity (needed for pure monocular VO)
  kNone     ///< no alignment (estimate already in the groundtruth frame, e.g. GPS-initialised)
};

struct AteResult
{
  bool valid = false;
  size_t num_matches = 0;
  double rmse = 0.0;
  double mean = 0.0;
  double median = 0.0;
  double max = 0.0;
  double scale = 1.0;
  double trajectory_length = 0.0;  ///< length of the matched GT trajectory [m]
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
};

/// Associates estimate and groundtruth (GT linearly interpolated at the
/// estimate timestamps) and computes the ATE after alignment.
AteResult computeAte(const StampedPoses& estimate, const StampedPoses& gt,
                     AlignmentType type,
                     StampedPoses* aligned_estimate = nullptr,
                     StampedPoses* matched_gt = nullptr);

std::string alignmentName(AlignmentType type);

} // namespace svo
