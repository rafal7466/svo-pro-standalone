#include "visualizer_cv.h"

#include <algorithm>
#include <cstdio>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <svo/common/frame.h>
#include <svo/common/point.h>
#include <svo/direct/feature_detection_utils.h>
#include <svo/frame_handler_base.h>
#include <svo/initialization.h>
#include <svo/map.h>
#include <svo/tracker/feature_tracking_utils.h>

#include "trajectory_eval.h"

namespace svo {

namespace {
const char* kTrackWindow = "SVO Pro - tracking";
const char* kTrajWindow = "SVO Pro - trajectory (top view)";

void putCaption(cv::Mat& img, const std::string& caption)
{
  if (caption.empty())
    return;
  int baseline = 0;
  const double scale = 0.5;
  cv::Size sz = cv::getTextSize(caption, cv::FONT_HERSHEY_SIMPLEX, scale, 1, &baseline);
  cv::rectangle(img, cv::Point(0, 0), cv::Point(std::min(img.cols, sz.width + 10), sz.height + 12),
                cv::Scalar(0, 0, 0), cv::FILLED);
  cv::putText(img, caption, cv::Point(5, sz.height + 5), cv::FONT_HERSHEY_SIMPLEX,
              scale, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
}
} // namespace

VisualizerCV::VisualizerCV(bool show_windows, const PoseEntries* groundtruth)
  : show_(show_windows), gt_(groundtruth)
{
  if (gt_ && gt_->empty())
    gt_ = nullptr;
  if (show_)
  {
    cv::namedWindow(kTrackWindow, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(kTrajWindow, cv::WINDOW_AUTOSIZE);
  }
}

void VisualizerCV::addPose(const PoseEntry& pose)
{
  estimate_.push_back(pose);
}

void VisualizerCV::update(const FrameHandlerBase& svo,
                          const std::vector<cv::Mat>& images,
                          const std::string& caption,
                          bool backend_fixed_to_global)
{
  ++update_count_;
  const FrameBundlePtr frames = svo.getLastFrames();

  // ---------------------------------------------------------------- images
  std::vector<cv::Mat> views;
  if (svo.stage() == Stage::kTracking && frames)
  {
    for (size_t i = 0; i < frames->size(); ++i)
    {
      cv::Mat img_rgb;
      feature_detection_utils::drawFeatures(*frames->at(i), 0, true, &img_rgb);
      if (backend_fixed_to_global)
        cv::rectangle(img_rgb, cv::Point(0, 0), cv::Point(img_rgb.cols, img_rgb.rows),
                      cv::Scalar(0, 255, 0), 6);
      views.push_back(img_rgb);
    }
  }
  else if (svo.stage() == Stage::kInitializing && frames && svo.initializer_ &&
           svo.initializer_->frames_ref_)
  {
    const FrameBundlePtr& ref = svo.initializer_->frames_ref_;
    for (size_t i = 0; i < std::min(ref->size(), frames->size()); ++i)
    {
      std::vector<std::pair<size_t, size_t>> matches;
      feature_tracking_utils::getFeatureMatches(*ref->at(i), *frames->at(i), &matches);
      cv::Mat img_rgb;
      cv::cvtColor(frames->at(i)->img(), img_rgb, cv::COLOR_GRAY2BGR);
      for (const auto& m : matches)
      {
        const auto& pr = ref->at(i)->px_vec_.col(static_cast<Eigen::Index>(m.first));
        const auto& pc = frames->at(i)->px_vec_.col(static_cast<Eigen::Index>(m.second));
        cv::line(img_rgb, cv::Point2f(static_cast<float>(pc(0)), static_cast<float>(pc(1))),
                 cv::Point2f(static_cast<float>(pr(0)), static_cast<float>(pr(1))),
                 cv::Scalar(0, 255, 0), 2);
      }
      views.push_back(img_rgb);
    }
  }
  if (views.empty())
  {
    for (const auto& im : images)
    {
      cv::Mat img_rgb;
      cv::cvtColor(im, img_rgb, cv::COLOR_GRAY2BGR);
      views.push_back(img_rgb);
    }
  }
  if (!views.empty())
  {
    cv::hconcat(views, track_canvas_);
    putCaption(track_canvas_, caption);
  }

  // ------------------------------------------------------------- local map
  if (svo.stage() == Stage::kTracking && (update_count_ % 5 == 0 || map_points_.empty()))
  {
    map_points_.clear();
    keyframes_.clear();
    const MapPtr& map = svo.map();
    if (map)
    {
      for (const auto& kv : map->keyframes_)
      {
        const FramePtr& kf = kv.second;
        if (!kf)
          continue;
        keyframes_.push_back(kf->imuPos());
        for (size_t j = 0; j < kf->landmark_vec_.size(); ++j)
        {
          const PointPtr& p = kf->landmark_vec_[j];
          if (p)
            map_points_.push_back(p->pos());
        }
      }
    }
  }

  if (show_)
  {
    drawTrajectory();
    if (!track_canvas_.empty())
      cv::imshow(kTrackWindow, track_canvas_);
    cv::imshow(kTrajWindow, traj_canvas_);
  }
}

void VisualizerCV::drawTrajectory()
{
  const int W = 700, H = 700, margin = 40;
  traj_canvas_ = cv::Mat(H, W, CV_8UC3, cv::Scalar(255, 255, 255));

  // update GT alignment from time to time
  if (gt_ && estimate_.size() > 50 &&
      (!have_alignment_ || estimate_.size() >= last_alignment_size_ + 50))
  {
    AteResult r = computeAte(estimate_, *gt_, AlignmentType::kPosYaw);
    if (r.valid)
    {
      // r: est -> gt. We need gt -> est
      R_est_gt_ = r.R.transpose();
      t_est_gt_ = -R_est_gt_ * r.t;
      have_alignment_ = true;
    }
    last_alignment_size_ = estimate_.size();
  }

  // bounds
  double xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9;
  auto extend = [&](const Eigen::Vector3d& p) {
    xmin = std::min(xmin, p.x()); xmax = std::max(xmax, p.x());
    ymin = std::min(ymin, p.y()); ymax = std::max(ymax, p.y());
  };
  for (const auto& e : estimate_) extend(e.p);
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> gt_pts;
  if (gt_ && have_alignment_)
  {
    const int64_t t0 = estimate_.empty() ? 0 : estimate_.front().timestamp_ns;
    const int64_t t1 = estimate_.empty() ? 0 : estimate_.back().timestamp_ns;
    for (size_t i = 0; i < gt_->size(); i += 10)
    {
      const auto& g = (*gt_)[i];
      if (g.timestamp_ns < t0 || g.timestamp_ns > t1)
        continue;
      gt_pts.push_back(R_est_gt_ * g.p + t_est_gt_);
      extend(gt_pts.back());
    }
  }
  if (xmin > xmax)
  {
    xmin = -1; xmax = 1; ymin = -1; ymax = 1;
  }
  const double span = std::max({xmax - xmin, ymax - ymin, 1.0});
  const double scale = (std::min(W, H) - 2.0 * margin) / span;
  const double cx = 0.5 * (xmin + xmax), cy = 0.5 * (ymin + ymax);
  auto toPx = [&](const Eigen::Vector3d& p) {
    return cv::Point(static_cast<int>(W / 2 + (p.x() - cx) * scale),
                     static_cast<int>(H / 2 - (p.y() - cy) * scale));
  };

  // grid (1 m)
  const double grid = span > 30 ? 5.0 : 1.0;
  for (double gx = std::floor((cx - span) / grid) * grid; gx < cx + span; gx += grid)
  {
    cv::line(traj_canvas_, toPx({gx, cy - span, 0}), toPx({gx, cy + span, 0}),
             cv::Scalar(235, 235, 235), 1);
  }
  for (double gy = std::floor((cy - span) / grid) * grid; gy < cy + span; gy += grid)
  {
    cv::line(traj_canvas_, toPx({cx - span, gy, 0}), toPx({cx + span, gy, 0}),
             cv::Scalar(235, 235, 235), 1);
  }

  for (const auto& p : map_points_)
    cv::circle(traj_canvas_, toPx(p), 1, cv::Scalar(160, 160, 160), cv::FILLED);
  for (size_t i = 1; i < gt_pts.size(); ++i)
    cv::line(traj_canvas_, toPx(gt_pts[i - 1]), toPx(gt_pts[i]), cv::Scalar(60, 60, 60), 1, cv::LINE_AA);
  for (size_t i = 1; i < estimate_.size(); ++i)
    cv::line(traj_canvas_, toPx(estimate_[i - 1].p), toPx(estimate_[i].p),
             cv::Scalar(0, 160, 0), 2, cv::LINE_AA);
  for (const auto& k : keyframes_)
    cv::circle(traj_canvas_, toPx(k), 3, cv::Scalar(255, 120, 0), 1, cv::LINE_AA);
  if (!estimate_.empty())
    cv::circle(traj_canvas_, toPx(estimate_.back().p), 5, cv::Scalar(0, 0, 255), cv::FILLED);

  char buf[200];
  std::snprintf(buf, sizeof(buf), "grid: %.0f m   green: SVO estimate   %s",
                grid, (gt_pts.empty() ? "" : "dark: groundtruth (pos-yaw aligned)"));
  cv::putText(traj_canvas_, buf, cv::Point(10, H - 12), cv::FONT_HERSHEY_SIMPLEX, 0.45,
              cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
}

int VisualizerCV::spin(int delay_ms)
{
  if (!show_)
    return -1;
  return cv::waitKey(delay_ms <= 0 ? 0 : delay_ms);
}

bool VisualizerCV::saveTrajectoryImage(const std::string& filename)
{
  // make sure the canvas reflects the final state
  drawTrajectory();
  if (traj_canvas_.empty())
    return false;
  return cv::imwrite(filename, traj_canvas_);
}

} // namespace svo
