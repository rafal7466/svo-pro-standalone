#pragma once
// Reader for datasets in the EuRoC / ASL format:
//   <root>[/mav0]/cam0/data.csv, cam0/data/<ts>.png
//   <root>[/mav0]/cam1/data.csv, cam1/data/<ts>.png      (stereo)
//   <root>[/mav0]/imu0/data.csv
//   <root>[/mav0]/state_groundtruth_estimate0/data.csv   (optional)

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <opencv2/core.hpp>

namespace svo {

struct ImageEntry
{
  int64_t timestamp_ns = 0;
  std::string path_cam0;
  std::string path_cam1;  // empty for mono
};

struct ImuEntry
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int64_t timestamp_ns = 0;
  Eigen::Vector3d gyr = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
};

struct PoseEntry
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int64_t timestamp_ns = 0;
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
};

using ImuEntries = std::vector<ImuEntry, Eigen::aligned_allocator<ImuEntry>>;
using PoseEntries = std::vector<PoseEntry, Eigen::aligned_allocator<PoseEntry>>;

class EurocDataset
{
public:
  /// @param root      dataset root folder (e.g. .../MH_01 or .../MH_01/mav0)
  /// @param stereo    also load cam1 and pair it with cam0
  bool load(const std::string& root, bool stereo);

  const std::vector<ImageEntry>& images() const { return images_; }
  const ImuEntries& imu() const { return imu_; }
  const PoseEntries& groundtruth() const { return gt_; }
  const std::string& dataRoot() const { return data_root_; }
  std::string name() const;

  /// Loads a gray-scale image (CV_8UC1). Returns empty Mat on failure.
  static cv::Mat loadImage(const std::string& path);

private:
  bool loadCameraCsv(const std::string& cam_dir,
                     std::vector<std::pair<int64_t, std::string>>& out) const;
  bool loadImuCsv(const std::string& file);
  bool loadGroundtruthCsv(const std::string& file);

  std::string root_;
  std::string data_root_;
  std::vector<ImageEntry> images_;
  ImuEntries imu_;
  PoseEntries gt_;
};

} // namespace svo
