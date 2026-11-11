#pragma once
// Lightweight OpenCV visualizer replacing the RViz based svo_ros Visualizer:
//  * "SVO Pro - tracking": camera image(s) with tracked features (or the
//    feature tracks while initializing), status caption
//  * "SVO Pro - trajectory": top-down (x-y) view of the estimated trajectory,
//    the active local map and (if available) the aligned groundtruth.

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "euroc_dataset.h"

namespace svo {

class FrameHandlerBase;

class VisualizerCV
{
public:
  VisualizerCV(bool show_windows, const PoseEntries* groundtruth);

  /// Update the visualization after a frame bundle has been processed.
  void update(const FrameHandlerBase& svo, const std::vector<cv::Mat>& images,
              const std::string& caption, bool backend_fixed_to_global = false);

  /// Add an estimated body pose to the trajectory plot.
  void addPose(const PoseEntry& pose);

  /// Process GUI events. Returns pressed key or -1.
  int spin(int delay_ms);

  /// Save the current trajectory canvas.
  bool saveTrajectoryImage(const std::string& filename);

  bool enabled() const { return show_; }

private:
  void drawTrajectory();

  bool show_ = true;
  const PoseEntries* gt_ = nullptr;
  PoseEntries estimate_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> map_points_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> keyframes_;
  // alignment GT -> estimate world frame (updated periodically)
  Eigen::Matrix3d R_est_gt_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_est_gt_ = Eigen::Vector3d::Zero();
  bool have_alignment_ = false;
  size_t last_alignment_size_ = 0;
  cv::Mat traj_canvas_;
  cv::Mat track_canvas_;
  size_t update_count_ = 0;
};

} // namespace svo
