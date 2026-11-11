#include "euroc_dataset.h"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

#include <glog/logging.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

namespace fs = std::filesystem;

namespace svo {

namespace {

void trim(std::string& s)
{
  while (!s.empty() && (s.back() == '\r' || s.back() == '\n' ||
                        s.back() == ' ' || s.back() == '\t'))
    s.pop_back();
  size_t i = 0;
  while (i < s.size() && (s[i] == ' ' || s[i] == '\t'))
    ++i;
  s.erase(0, i);
}

std::vector<std::string> splitCsv(const std::string& line)
{
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, ','))
  {
    trim(item);
    out.push_back(item);
  }
  return out;
}

bool isHiddenMacFile(const std::string& name)
{
  return name.size() >= 2 && name[0] == '.' && name[1] == '_';
}

} // namespace

std::string EurocDataset::name() const
{
  fs::path p(root_);
  if (p.filename().empty())
    p = p.parent_path();
  if (p.filename() == "mav0")
    p = p.parent_path();
  return p.filename().string();
}

bool EurocDataset::loadCameraCsv(
    const std::string& cam_dir,
    std::vector<std::pair<int64_t, std::string>>& out) const
{
  const fs::path csv = fs::path(cam_dir) / "data.csv";
  std::ifstream f(csv);
  if (!f.is_open())
  {
    LOG(ERROR) << "Cannot open " << csv.string();
    return false;
  }
  std::string line;
  while (std::getline(f, line))
  {
    trim(line);
    if (line.empty() || line[0] == '#')
      continue;
    auto tok = splitCsv(line);
    if (tok.size() < 2 || isHiddenMacFile(tok[1]))
      continue;
    const int64_t ts = std::strtoll(tok[0].c_str(), nullptr, 10);
    // the filename may also be an absolute path (datasets converted without
    // copying the images)
    const fs::path img(tok[1]);
    out.emplace_back(ts, img.is_absolute() ? img.string()
                                           : (fs::path(cam_dir) / "data" / img).string());
  }
  std::sort(out.begin(), out.end());
  return !out.empty();
}

bool EurocDataset::loadImuCsv(const std::string& file)
{
  std::ifstream f(file);
  if (!f.is_open())
  {
    LOG(ERROR) << "Cannot open " << file;
    return false;
  }
  std::string line;
  while (std::getline(f, line))
  {
    trim(line);
    if (line.empty() || line[0] == '#')
      continue;
    auto tok = splitCsv(line);
    if (tok.size() < 7)
      continue;
    ImuEntry e;
    e.timestamp_ns = std::strtoll(tok[0].c_str(), nullptr, 10);
    e.gyr = Eigen::Vector3d(std::atof(tok[1].c_str()), std::atof(tok[2].c_str()),
                            std::atof(tok[3].c_str()));
    e.acc = Eigen::Vector3d(std::atof(tok[4].c_str()), std::atof(tok[5].c_str()),
                            std::atof(tok[6].c_str()));
    imu_.push_back(e);
  }
  std::sort(imu_.begin(), imu_.end(), [](const ImuEntry& a, const ImuEntry& b) {
    return a.timestamp_ns < b.timestamp_ns;
  });
  return !imu_.empty();
}

bool EurocDataset::loadGroundtruthCsv(const std::string& file)
{
  std::ifstream f(file);
  if (!f.is_open())
    return false;
  std::string line;
  while (std::getline(f, line))
  {
    trim(line);
    if (line.empty() || line[0] == '#')
      continue;
    auto tok = splitCsv(line);
    if (tok.size() < 8)
      continue;
    PoseEntry e;
    e.timestamp_ns = std::strtoll(tok[0].c_str(), nullptr, 10);
    e.p = Eigen::Vector3d(std::atof(tok[1].c_str()), std::atof(tok[2].c_str()),
                          std::atof(tok[3].c_str()));
    // EuRoC: q_RS_w, q_RS_x, q_RS_y, q_RS_z
    e.q = Eigen::Quaterniond(std::atof(tok[4].c_str()), std::atof(tok[5].c_str()),
                             std::atof(tok[6].c_str()), std::atof(tok[7].c_str()));
    e.q.normalize();
    gt_.push_back(e);
  }
  return !gt_.empty();
}

bool EurocDataset::load(const std::string& root, bool stereo)
{
  root_ = root;
  fs::path r(root);
  if (fs::exists(r / "mav0" / "cam0"))
    r = r / "mav0";
  data_root_ = r.string();
  if (!fs::exists(r / "cam0"))
  {
    LOG(ERROR) << "Dataset folder " << r.string()
               << " does not contain cam0/. Expected EuRoC/ASL layout.";
    return false;
  }

  std::vector<std::pair<int64_t, std::string>> cam0, cam1;
  if (!loadCameraCsv((r / "cam0").string(), cam0))
    return false;
  if (stereo && !loadCameraCsv((r / "cam1").string(), cam1))
  {
    LOG(ERROR) << "Stereo requested but cam1 could not be loaded.";
    return false;
  }

  images_.clear();
  if (!stereo)
  {
    for (const auto& c : cam0)
    {
      ImageEntry e;
      e.timestamp_ns = c.first;
      e.path_cam0 = c.second;
      images_.push_back(e);
    }
  }
  else
  {
    // pair by timestamp (tolerance 2 ms)
    size_t j = 0;
    const int64_t tol = 2000000;
    for (const auto& c : cam0)
    {
      while (j < cam1.size() && cam1[j].first < c.first - tol)
        ++j;
      if (j < cam1.size() && std::llabs(cam1[j].first - c.first) <= tol)
      {
        ImageEntry e;
        e.timestamp_ns = c.first;
        e.path_cam0 = c.second;
        e.path_cam1 = cam1[j].second;
        images_.push_back(e);
        ++j;
      }
    }
  }

  if (!loadImuCsv((r / "imu0" / "data.csv").string()))
    LOG(WARNING) << "No IMU data loaded (imu0/data.csv).";

  const fs::path gt_file = r / "state_groundtruth_estimate0" / "data.csv";
  if (!loadGroundtruthCsv(gt_file.string()))
    LOG(WARNING) << "No groundtruth found (" << gt_file.string() << ").";

  LOG(INFO) << "Dataset " << data_root_ << ": " << images_.size()
            << (stereo ? " stereo" : " mono") << " frames, " << imu_.size()
            << " IMU samples, " << gt_.size() << " groundtruth poses.";
  return !images_.empty();
}

cv::Mat EurocDataset::loadImage(const std::string& path)
{
  return cv::imread(path, cv::IMREAD_GRAYSCALE);
}

} // namespace svo
