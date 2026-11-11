// =============================================================================
//  svo_euroc - standalone (ROS-free) runner for SVO Pro on EuRoC/ASL datasets
//
//  Replaces svo_ros (svo_node + SvoInterface + Visualizer + benchmark_node).
//  Images and IMU are read from disk in the EuRoC folder layout and fed to the
//  pipeline in timestamp order; IMU is always provided ahead of each image,
//  exactly as it would arrive from a ROS bag.
//
//  Example (Windows):
//    svo_euroc.exe --dataset=C:\data\MH_01 --config=config\euroc_vio_stereo.yaml
// =============================================================================

#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <thread>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <svo/ceres_backend_interface.hpp>
#include <svo/ceres_backend_publisher.hpp>
#include <svo/common/conversions.h>
#include <svo/common/frame.h>
#include <svo/direct/depth_filter.h>
#include <svo/frame_handler_base.h>
#include <svo/frame_handler_mono.h>
#include <svo/frame_handler_stereo.h>
#include <svo/imu_handler.h>

#include "euroc_dataset.h"
#include "param_server.h"
#include "svo_factory.h"
#include "trajectory_eval.h"
#include "visualizer_cv.h"

#ifdef _WIN32
#include <windows.h>
#endif

namespace fs = std::filesystem;

DEFINE_string(dataset, "", "Path to the EuRoC sequence (folder containing cam0/ "
                           "imu0/ ..., or the folder containing mav0/).");
DEFINE_string(config, "", "SVO parameter file, e.g. config/euroc_vio_stereo.yaml");
DEFINE_string(calib, "", "Camera(+IMU) calibration file. Overrides 'calib_file' "
                         "from the config. Relative paths are resolved w.r.t. "
                         "the config folder.");
DEFINE_string(pipeline, "", "mono | stereo. Default: from 'pipeline_is_stereo'.");
DEFINE_string(output_dir, "results", "Folder for trajectories / statistics.");
DEFINE_string(params, "", "Comma separated parameter overrides, e.g. "
                          "\"max_fts=150,use_imu=false\"");
DEFINE_bool(viz, true, "Show OpenCV visualization windows.");
DEFINE_bool(wait_at_end, true, "With --viz: keep the windows open at the end "
                               "until a key is pressed.");
DEFINE_int32(start_frame, 0, "Index of the first image to process.");
DEFINE_int32(max_frames, -1, "Maximum number of images to process (-1: all).");
DEFINE_bool(sync_threads, true, "After every frame wait until the depth-filter "
                                "thread and the backend optimization are idle. "
                                "Emulates real-time processing on a fast machine "
                                "while running as fast as possible. Set to false "
                                "together with --realtime_factor=1 to reproduce "
                                "the asynchronous ROS behaviour.");
DEFINE_double(realtime_factor, 0.0, "Playback speed: 0 = as fast as possible, "
                                    "1 = real-time.");
DEFINE_double(imu_lookahead, 0.02, "IMU data [s] provided ahead of each image.");
DEFINE_string(init_velocity, "", "Initial velocity \"vx,vy,vz\" [m/s] of the first "
                                 "backend state in the IMU (body) frame, e.g. for a "
                                 "start in flight.");
DEFINE_string(init_velocity_file, "", "CSV with a body-frame velocity per image "
                                      "(columns: index,timestamp_ns,...,vx,vy,vz as the "
                                      "last three). The value at the frame where the "
                                      "backend starts is used as initial velocity "
                                      "(hand-off from GPS/INS).");
DEFINE_double(init_velocity_sigma, 2.0, "Prior sigma [m/s] of the initial velocity.");
DEFINE_double(gps_aiding_duration, -1.0, "GPS aiding: the groundtruth positions of "
                                         "the dataset are fused as GPS in the backend "
                                         "for this many seconds after the first "
                                         "processed image, then the navigation "
                                         "continues GPS-denied (-1: off).");
DEFINE_double(gps_sigma_h, 2.0, "GPS horizontal position sigma [m].");
DEFINE_double(gps_sigma_v, 4.0, "GPS vertical position sigma [m].");
DEFINE_double(image_scale, 1.0, "Resize the input images by this factor before "
                                "processing (the calibration must match the "
                                "resized images).");
DEFINE_double(clahe, 0.0, "If > 0: contrast enhancement (CLAHE, this clip limit, "
                          "8x8 tiles) of the input images, e.g. 2.0 for hazy / "
                          "low-contrast aerial images.");

namespace svo {

enum class PipelineType { kMono, kStereo };

// -----------------------------------------------------------------------------
// SvoRunner: port of svo_ros/SvoInterface without ROS
// -----------------------------------------------------------------------------
class SvoRunner
{
public:
  SvoRunner(PipelineType type, const ParamServer& params)
    : params_(params), pipeline_type_(type)
  {
    set_initial_attitude_from_gravity_ =
        vk::param<bool>(params_, "set_initial_attitude_from_gravity", true);
    automatic_reinitialization_ =
        vk::param<bool>(params_, "automatic_reinitialization", false);

    if (type == PipelineType::kMono)
      svo_ = factory::makeMono(params_);
    else
      svo_ = factory::makeStereo(params_);
    ncam_ = svo_->getNCamera();

    if (vk::param<bool>(params_, "use_imu", false))
    {
      imu_handler_ = factory::getImuHandler(params_);
      svo_->imu_handler_ = imu_handler_;
    }
    createBackend();
    if (vk::param<bool>(params_, "runlc", false))
      LOG(WARNING) << "Loop closing (runlc) is not part of this standalone build "
                      "and is ignored.";
    if (vk::param<bool>(params_, "use_global_map", false))
      LOG(WARNING) << "Global map (iSAM2/GTSAM) is not part of this standalone "
                      "build and is ignored.";
    svo_->start();
  }

  ~SvoRunner()
  {
    if (ceres_backend_interface_)
      ceres_backend_interface_->quitThread();
  }

  void createBackend()
  {
    if (!vk::param<bool>(params_, "use_ceres_backend", false))
      return;
    if (!imu_handler_)
    {
      LOG(ERROR) << "Cannot use ceres backend without using imu";
      return;
    }
    ceres_backend_interface_ = ceres_backend_factory::makeBackend(params_, ncam_);
    svo_->setBundleAdjuster(ceres_backend_interface_);
    ceres_backend_interface_->setImu(imu_handler_);
    ceres_backend_interface_->makePublisher(ceres_backend_publisher_);
    if (backend_state_cb_)
      ceres_backend_publisher_->setStateCallback(backend_state_cb_);
    if (gps_provider_)
      ceres_backend_interface_->setGpsProvider(gps_provider_);
  }

  void setGpsProvider(const CeresBackendInterface::GpsProvider& p)
  {
    gps_provider_ = p;
    if (ceres_backend_interface_)
      ceres_backend_interface_->setGpsProvider(p);
  }

  void setBackendStateCallback(const CeresBackendPublisher::StateCallback& cb)
  {
    backend_state_cb_ = cb;
    if (ceres_backend_publisher_)
      ceres_backend_publisher_->setStateCallback(cb);
  }

  void addImu(const ImuEntry& m)
  {
    if (!imu_handler_)
      return;
    const ImuMeasurement meas(static_cast<double>(m.timestamp_ns) * 1e-9, m.gyr, m.acc);
    imu_handler_->addImuMeasurement(meas);
  }

  /// Same logic as SvoInterface::setImuPrior
  bool setImuPrior(const int64_t timestamp_nanoseconds)
  {
    if (svo_->getBundleAdjuster())
    {
      if (!svo_->hasStarted())
      {
        if (imu_handler_->getMeasurementsCopy().size() < 10u)
          return false;
      }
      return true;
    }
    if (imu_handler_ && !svo_->hasStarted() && set_initial_attitude_from_gravity_)
    {
      Quaternion R_imu_world;
      if (imu_handler_->getInitialAttitude(
              timestamp_nanoseconds * common::conversions::kNanoSecondsToSeconds,
              R_imu_world))
      {
        svo_->setRotationPrior(R_imu_world);
      }
      else
      {
        return false;
      }
    }
    else if (imu_handler_ && svo_->getLastFrames())
    {
      Quaternion R_lastimu_newimu;
      if (imu_handler_->getRelativeRotationPrior(
              svo_->getLastFrames()->getMinTimestampNanoseconds() *
                  common::conversions::kNanoSecondsToSeconds,
              timestamp_nanoseconds * common::conversions::kNanoSecondsToSeconds,
              false, R_lastimu_newimu))
      {
        svo_->setRotationIncrementPrior(R_lastimu_newimu);
      }
    }
    return true;
  }

  /// Returns false if the frame was skipped (IMU prior not yet available).
  bool processImages(const std::vector<cv::Mat>& images, int64_t timestamp_ns)
  {
    if (imu_handler_ && !setImuPrior(timestamp_ns))
    {
      VLOG(3) << "Could not align gravity! Attempting again in next iteration.";
      return false;
    }
    if (!svo_->isBackendValid())
    {
      // backend was reset (e.g. after re-initialization)
      if (vk::param<bool>(params_, "use_ceres_backend", false))
        createBackend();
    }
    if (ceres_backend_interface_ && ceres_backend_interface_->getNumFrames() == 0 &&
        init_velocity_provider_)
    {
      Eigen::Vector3d v_body;
      if (init_velocity_provider_(timestamp_ns, &v_body))
        ceres_backend_interface_->setInitialVelocityBody(v_body, FLAGS_init_velocity_sigma);
    }
    svo_->addImageBundle(images, static_cast<uint64_t>(timestamp_ns));

    if (FLAGS_sync_threads)
    {
      if (svo_->depth_filter_)
        svo_->depth_filter_->waitUntilIdle(5.0);
      if (svo_->isBackendValid() && ceres_backend_interface_)
        ceres_backend_interface_->waitForOptimization(5.0);
    }

    if (svo_->stage() == Stage::kPaused && automatic_reinitialization_)
      svo_->start();
    return true;
  }

  std::string statusCaption() const
  {
    std::ostringstream ss;
    ss << svo_->stageStr() << " | " << svo_->trackingQualityStr();
    if (svo_->getLastFrames())
      ss << " | fts: " << svo_->getLastFrames()->numTrackedLandmarks()
         << " | kfs: " << svo_->map()->size();
    ss << " | " << std::fixed << std::setprecision(1)
       << svo_->lastProcessingTime() * 1000.0 << " ms";
    if (svo_->isBackendValid() && ceres_backend_interface_)
    {
      const std::string st = ceres_backend_interface_->getStationaryStatusStr();
      if (!st.empty())
        ss << " | " << st;
    }
    return ss.str();
  }

  const ParamServer& params_;
  PipelineType pipeline_type_;
  std::shared_ptr<FrameHandlerBase> svo_;
  std::shared_ptr<ImuHandler> imu_handler_;
  std::shared_ptr<CeresBackendInterface> ceres_backend_interface_;
  std::shared_ptr<CeresBackendPublisher> ceres_backend_publisher_;
  CeresBackendPublisher::StateCallback backend_state_cb_;
  std::function<bool(int64_t, Eigen::Vector3d*)> init_velocity_provider_;
  CeresBackendInterface::GpsProvider gps_provider_;
  CameraBundlePtr ncam_;
  bool set_initial_attitude_from_gravity_ = true;
  bool automatic_reinitialization_ = false;
};

// -----------------------------------------------------------------------------
std::string resolvePath(const std::string& p, const fs::path& base_dir)
{
  if (p.empty())
    return p;
  fs::path path(p);
  if (path.is_absolute() || fs::exists(path))
    return path.string();
  fs::path alt = base_dir / path;
  if (fs::exists(alt))
    return alt.string();
  return path.string();
}

PoseEntry toPoseEntry(int64_t ts, const Transformation& T)
{
  PoseEntry p;
  p.timestamp_ns = ts;
  p.p = T.getPosition();
  p.q = T.getEigenQuaternion();
  return p;
}

void printAte(std::ostream& out, const std::string& label, const AteResult& r,
              AlignmentType type)
{
  out << "  " << std::left << std::setw(10) << label << std::setw(7)
      << alignmentName(type) << std::right << std::fixed << std::setprecision(4)
      << " RMSE " << std::setw(8) << r.rmse << " m  mean " << std::setw(8) << r.mean
      << "  median " << std::setw(8) << r.median << "  max " << std::setw(8) << r.max;
  if (type == AlignmentType::kSim3)
    out << "  scale " << std::setprecision(4) << r.scale;
  out << "  (" << r.num_matches << " poses)\n";
}

} // namespace svo

int main(int argc, char** argv)
{
  using namespace svo;
  gflags::SetUsageMessage(
      "svo_euroc --dataset=<EuRoC sequence folder> --config=<params.yaml> "
      "[--calib=<calib.yaml>] [--pipeline=mono|stereo] [--viz=true]");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
#ifdef _WIN32
  // enable ANSI colour codes used by SVO's log macros in the Windows console
  {
    HANDLE h = GetStdHandle(STD_ERROR_HANDLE);
    DWORD mode = 0;
    if (h != INVALID_HANDLE_VALUE && GetConsoleMode(h, &mode))
      SetConsoleMode(h, mode | 0x0004 /*ENABLE_VIRTUAL_TERMINAL_PROCESSING*/);
  }
#endif
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);

  if (FLAGS_dataset.empty() || FLAGS_config.empty())
  {
    std::cerr << gflags::ProgramUsage() << std::endl
              << "Both --dataset and --config are required." << std::endl;
    return 1;
  }

  // ---------------------------------------------------------------- params
  ParamServer params;
  if (!params.loadYaml(FLAGS_config))
    return 1;
  const fs::path config_dir = fs::absolute(fs::path(FLAGS_config)).parent_path();
  {
    std::stringstream ss(FLAGS_params);
    std::string item;
    while (std::getline(ss, item, ','))
      if (!item.empty() && !params.setFromAssignment(item))
        LOG(WARNING) << "Ignoring malformed --params entry '" << item << "'";
  }
  std::string calib = FLAGS_calib.empty()
                          ? params.get<std::string>("calib_file", "")
                          : FLAGS_calib;
  calib = resolvePath(calib, config_dir);
  if (calib.empty() || !fs::exists(calib))
  {
    LOG(ERROR) << "Calibration file not found: '" << calib
               << "'. Use --calib or set calib_file in the config.";
    return 1;
  }
  params.set("calib_file", calib);

  bool stereo = params.get<bool>("pipeline_is_stereo", false);
  if (FLAGS_pipeline == "mono")
    stereo = false;
  else if (FLAGS_pipeline == "stereo")
    stereo = true;
  else if (!FLAGS_pipeline.empty())
  {
    LOG(ERROR) << "Unknown --pipeline " << FLAGS_pipeline;
    return 1;
  }

  // ---------------------------------------------------------------- dataset
  EurocDataset dataset;
  if (!dataset.load(FLAGS_dataset, stereo))
    return 1;

  const std::string run_name =
      dataset.name() + (stereo ? "_stereo" : "_mono") +
      (params.get<bool>("use_imu", false) ? "_vio" : "_vo");
  const fs::path out_dir = fs::path(FLAGS_output_dir) / run_name;
  fs::create_directories(out_dir);
  if (!params.has("trace_dir"))
    params.set("trace_dir", out_dir.string());

  // ---------------------------------------------------------------- pipeline
  std::unique_ptr<SvoRunner> runner(
      new SvoRunner(stereo ? PipelineType::kStereo : PipelineType::kMono, params));
  params.printUsed(std::cout);

  // ---- optional initial velocity of the backend (start in flight) ----------
  {
    std::vector<std::pair<int64_t, Eigen::Vector3d>> vel_table;
    if (!FLAGS_init_velocity_file.empty())
    {
      std::ifstream vf(FLAGS_init_velocity_file);
      if (!vf.is_open())
      {
        LOG(ERROR) << "Cannot open --init_velocity_file " << FLAGS_init_velocity_file;
        return 1;
      }
      std::string line;
      while (std::getline(vf, line))
      {
        if (line.empty() || line[0] == '#')
          continue;
        std::vector<std::string> tok;
        std::stringstream ls(line);
        std::string t;
        while (std::getline(ls, t, ','))
          tok.push_back(t);
        if (tok.size() < 5)
          continue;
        const size_t n = tok.size();
        vel_table.emplace_back(std::strtoll(tok[1].c_str(), nullptr, 10),
                               Eigen::Vector3d(std::atof(tok[n - 3].c_str()),
                                               std::atof(tok[n - 2].c_str()),
                                               std::atof(tok[n - 1].c_str())));
      }
      std::sort(vel_table.begin(), vel_table.end(),
                [](const auto& a, const auto& b) { return a.first < b.first; });
      LOG(INFO) << "Loaded " << vel_table.size() << " initial-velocity samples.";
    }
    Eigen::Vector3d v_fixed = Eigen::Vector3d::Zero();
    bool have_fixed = false;
    if (!FLAGS_init_velocity.empty())
    {
      std::stringstream vs(FLAGS_init_velocity);
      std::string t;
      int k = 0;
      while (std::getline(vs, t, ',') && k < 3)
        v_fixed[k++] = std::atof(t.c_str());
      have_fixed = (k == 3);
    }
    if (have_fixed || !vel_table.empty())
    {
      runner->init_velocity_provider_ =
          [vel_table, v_fixed, have_fixed](int64_t ts, Eigen::Vector3d* v) {
            if (have_fixed)
            {
              *v = v_fixed;
              return true;
            }
            auto it = std::lower_bound(
                vel_table.begin(), vel_table.end(), ts,
                [](const std::pair<int64_t, Eigen::Vector3d>& a, int64_t t) {
                  return a.first < t;
                });
            if (it == vel_table.end())
              --it;
            if (it != vel_table.begin() &&
                std::llabs((it - 1)->first - ts) < std::llabs(it->first - ts))
              --it;
            if (std::llabs(it->first - ts) > 200000000LL)  // > 0.2 s
              return false;
            *v = it->second;
            return true;
          };
    }
  }

  StampedPoses traj_frontend;
  StampedPoses traj_backend;
  // t, v_W, gyro bias, accel bias of the backend states
  std::vector<std::array<double, 10>> backend_speed_bias;
  std::mutex backend_traj_mutex;
  runner->setBackendStateCallback(
      [&](const ViNodeState& state, int64_t ts, int32_t /*bundle_id*/) {
        std::lock_guard<std::mutex> lock(backend_traj_mutex);
        if (!traj_backend.empty() && traj_backend.back().timestamp_ns == ts)
          traj_backend.back() = toPoseEntry(ts, state.get_T_W_B());
        else
          traj_backend.push_back(toPoseEntry(ts, state.get_T_W_B()));
        std::array<double, 10> sb = {static_cast<double>(ts) * 1e-9,
            state.get_W_v_B().x(), state.get_W_v_B().y(), state.get_W_v_B().z(),
            state.getGyroBias().x(), state.getGyroBias().y(), state.getGyroBias().z(),
            state.getAccBias().x(), state.getAccBias().y(), state.getAccBias().z()};
        if (!backend_speed_bias.empty() && backend_speed_bias.back()[0] == sb[0])
          backend_speed_bias.back() = sb;
        else
          backend_speed_bias.push_back(sb);
      });

  // ---- optional GPS aiding (positions of the dataset groundtruth) ----------
  double gps_end_time = -1.0;
  if (FLAGS_gps_aiding_duration > 0.0)
  {
    const auto& gt = dataset.groundtruth();
    const size_t first_img = static_cast<size_t>(std::max(0, FLAGS_start_frame));
    if (gt.size() < 10 || first_img >= dataset.images().size())
    {
      LOG(ERROR) << "--gps_aiding_duration needs groundtruth positions in the dataset.";
      return 1;
    }
    const double t0 = static_cast<double>(dataset.images()[first_img].timestamp_ns) * 1e-9;
    gps_end_time = t0 + FLAGS_gps_aiding_duration;
    const double sh = FLAGS_gps_sigma_h, sv = FLAGS_gps_sigma_v;
    const PoseEntries* gtp = &gt;
    auto interp = [gtp](double t, Eigen::Vector3d* p) {
      const int64_t ts = static_cast<int64_t>(t * 1e9);
      const auto& g = *gtp;
      auto it = std::lower_bound(g.begin(), g.end(), ts,
                                 [](const PoseEntry& a, int64_t x) { return a.timestamp_ns < x; });
      if (it == g.begin() || it == g.end())
        return false;
      auto pr = it - 1;
      const double dt = static_cast<double>(it->timestamp_ns - pr->timestamp_ns);
      if (dt <= 0.0 || dt > 0.5e9)
        return false;
      const double a = static_cast<double>(ts - pr->timestamp_ns) / dt;
      *p = (1.0 - a) * pr->p + a * it->p;
      return true;
    };
    runner->setGpsProvider(
        [interp, gps_end_time, sh, sv](double t, CeresBackendInterface::GpsSample* s) {
          if (t > gps_end_time)
            return false;
          Eigen::Vector3d p, pa, pb;
          if (!interp(t, &p) || !interp(t - 0.5, &pa) || !interp(t + 0.5, &pb))
          {
            LOG_EVERY_N(WARNING, 200) << "No GPS position at t = " << std::fixed << t;
            return false;
          }
          s->p_W = p;
          s->v_W = (pb - pa) / 1.0;
          s->sigma_h = sh;
          s->sigma_v = sv;
          return true;
        });
    LOG(INFO) << "GPS aiding for " << FLAGS_gps_aiding_duration
              << " s (until t = " << std::fixed << gps_end_time
              << "), then GPS-denied. World frame = groundtruth (ENU) frame.";
  }

  VisualizerCV vis(FLAGS_viz, &dataset.groundtruth());

  const auto& images = dataset.images();
  const auto& imu = dataset.imu();
  size_t imu_idx = 0;
  const size_t first = static_cast<size_t>(std::max(0, FLAGS_start_frame));
  size_t last = images.size();
  if (FLAGS_max_frames > 0)
    last = std::min(last, first + static_cast<size_t>(FLAGS_max_frames));

  // skip IMU before the first processed frame (keep 1 s of history)
  if (first < images.size())
  {
    const int64_t t_first = images[first].timestamp_ns - 1000000000LL;
    while (imu_idx < imu.size() && imu[imu_idx].timestamp_ns < t_first)
      ++imu_idx;
  }

  cv::Ptr<cv::CLAHE> clahe;
  if (FLAGS_clahe > 0.0)
    clahe = cv::createCLAHE(FLAGS_clahe, cv::Size(8, 8));

  std::vector<double> proc_times;
  size_t n_tracking = 0, n_skipped = 0, n_reinit = 0;
  Stage last_stage = Stage::kPaused;
  bool quit = false, paused = false;
  const auto wall_start = std::chrono::steady_clock::now();
  const int64_t lookahead_ns = static_cast<int64_t>(FLAGS_imu_lookahead * 1e9);

  std::cout << "Processing frames " << first << " .. " << last << " of "
            << images.size() << (FLAGS_viz ? "  (press 'q' to quit, SPACE to pause)" : "")
            << std::endl;

  for (size_t i = first; i < last && !quit; ++i)
  {
    const ImageEntry& entry = images[i];
    const auto t_frame_start = std::chrono::steady_clock::now();

    // feed IMU up to image time + lookahead
    while (imu_idx < imu.size() &&
           imu[imu_idx].timestamp_ns <= entry.timestamp_ns + lookahead_ns)
    {
      runner->addImu(imu[imu_idx]);
      ++imu_idx;
    }

    std::vector<cv::Mat> imgs;
    imgs.push_back(EurocDataset::loadImage(entry.path_cam0));
    if (stereo)
      imgs.push_back(EurocDataset::loadImage(entry.path_cam1));
    for (auto& im : imgs)
    {
      if (im.empty())
        continue;
      if (FLAGS_image_scale > 0.0 && FLAGS_image_scale != 1.0)
        cv::resize(im, im, cv::Size(), FLAGS_image_scale, FLAGS_image_scale,
                   FLAGS_image_scale < 1.0 ? cv::INTER_AREA : cv::INTER_LINEAR);
      if (clahe)
        clahe->apply(im, im);
    }
    bool ok = true;
    for (const auto& im : imgs)
      ok = ok && !im.empty();
    if (!ok)
    {
      LOG(ERROR) << "Could not read image(s) of frame " << i << ": " << entry.path_cam0;
      continue;
    }

    const auto t0 = std::chrono::steady_clock::now();
    const bool processed = runner->processImages(imgs, entry.timestamp_ns);
    const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (!processed)
    {
      ++n_skipped;
      continue;
    }
    proc_times.push_back(dt);

    const Stage stage = runner->svo_->stage();
    if (stage == Stage::kTracking && runner->svo_->getLastFrames())
    {
      ++n_tracking;
      PoseEntry p = toPoseEntry(entry.timestamp_ns,
                                runner->svo_->getLastFrames()->get_T_W_B());
      traj_frontend.push_back(p);
      vis.addPose(p);
    }
    if (last_stage == Stage::kTracking && stage != Stage::kTracking)
    {
      ++n_reinit;
      LOG(WARNING) << "Tracking lost at frame " << i << " -> " << runner->svo_->stageStr();
    }
    last_stage = stage;

    if (FLAGS_viz)
    {
      bool fixed = false;
      if (runner->svo_->isBackendValid())
        fixed = runner->svo_->getBundleAdjuster()->isFixedToGlobalMap();
      std::ostringstream cap;
      cap << "#" << i << " | " << runner->statusCaption();
      vis.update(*runner->svo_, imgs, cap.str(), fixed);
      int key = vis.spin(1);
      do
      {
        if (key == 'q' || key == 27)
          quit = true;
        else if (key == ' ')
          paused = !paused;
        else if (key == 'r')
        {
          runner->svo_->reset();
          runner->svo_->start();
        }
        if (paused && !quit)
          key = vis.spin(30);
      } while (paused && !quit);
    }

    if ((i - first) % 100 == 0)
    {
      std::cout << "[" << std::setw(5) << i << "/" << images.size() << "] "
                << runner->statusCaption() << std::endl;
    }

    if (FLAGS_realtime_factor > 0.0 && i + 1 < last)
    {
      const double frame_dt =
          static_cast<double>(images[i + 1].timestamp_ns - entry.timestamp_ns) * 1e-9 /
          FLAGS_realtime_factor;
      const double used = std::chrono::duration<double>(
                              std::chrono::steady_clock::now() - t_frame_start).count();
      if (frame_dt > used)
        std::this_thread::sleep_for(std::chrono::duration<double>(frame_dt - used));
    }
  }

  const double wall_time = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - wall_start).count();

  // stop the backend thread before reading its trajectory
  if (runner->ceres_backend_interface_)
  {
    runner->ceres_backend_interface_->waitForOptimization(5.0);
    runner->ceres_backend_interface_->quitThread();
  }

  // ---------------------------------------------------------------- results
  StampedPoses traj_backend_copy;
  {
    std::lock_guard<std::mutex> lock(backend_traj_mutex);
    traj_backend_copy = traj_backend;
  }
  writeTumTrajectory((out_dir / "stamped_traj_estimate.txt").string(), traj_frontend);
  writeEurocTrajectory((out_dir / "traj_estimate_euroc.csv").string(), traj_frontend);
  if (!traj_backend_copy.empty())
  {
    writeTumTrajectory((out_dir / "stamped_traj_backend.txt").string(), traj_backend_copy);
    std::lock_guard<std::mutex> lock(backend_traj_mutex);
    std::ofstream f(out_dir / "stamped_backend_speed_bias.txt");
    f << "# timestamp vx vy vz bg_x bg_y bg_z ba_x ba_y ba_z (world velocity, IMU biases)\n"
      << std::fixed << std::setprecision(9);
    for (const auto& r : backend_speed_bias)
    {
      for (size_t k = 0; k < r.size(); ++k)
        f << r[k] << (k + 1 < r.size() ? " " : "\n");
    }
  }
  if (!dataset.groundtruth().empty())
    writeTumTrajectory((out_dir / "stamped_groundtruth.txt").string(), dataset.groundtruth());

  std::ostringstream summary;
  summary << "================ SVO Pro standalone - summary ================\n";
  summary << "  dataset       : " << dataset.dataRoot() << "\n";
  summary << "  pipeline      : " << (stereo ? "stereo" : "mono")
          << (params.get<bool>("use_imu", false) ? " + IMU" : "")
          << (runner->ceres_backend_interface_ ? " + ceres VIO backend" : "") << "\n";
  summary << "  frames        : " << (last - first) << " (tracking " << n_tracking
          << ", skipped " << n_skipped << ", tracking lost " << n_reinit << "x)\n";
  if (!proc_times.empty())
  {
    std::vector<double> s = proc_times;
    std::sort(s.begin(), s.end());
    const double mean = std::accumulate(s.begin(), s.end(), 0.0) / static_cast<double>(s.size());
    summary << std::fixed << std::setprecision(2)
            << "  time / frame  : mean " << mean * 1000.0 << " ms, median "
            << s[s.size() / 2] * 1000.0 << " ms (frontend"
            << (FLAGS_sync_threads ? " + wait for depth-filter/backend threads" : "")
            << ")\n";
  }
  summary << std::fixed << std::setprecision(1) << "  wall time     : " << wall_time << " s\n";
  summary << "  output        : " << fs::absolute(out_dir).string() << "\n";

  if (!dataset.groundtruth().empty())
  {
    summary << "  ------------- Absolute Trajectory Error (ATE) -------------\n";
    const bool use_imu = params.get<bool>("use_imu", false);
    // metric scale is only observable with stereo or with the visual-inertial backend
    const bool mono_vo = !stereo && !(use_imu && runner->ceres_backend_interface_);
    // Without IMU the world frame is not gravity aligned -> pos-yaw makes no sense
    std::vector<AlignmentType> types;
    if (gps_end_time > 0.0)
      types.push_back(AlignmentType::kNone);
    if (use_imu)
      types.push_back(AlignmentType::kPosYaw);
    types.push_back(AlignmentType::kSE3);
    types.push_back(AlignmentType::kSim3);
    const AlignmentType save_type =
        mono_vo ? AlignmentType::kSim3
                : (use_imu ? AlignmentType::kPosYaw : AlignmentType::kSE3);
    struct Item { std::string label; const StampedPoses* traj; };
    std::vector<Item> items = {{"frontend", &traj_frontend}};
    if (!traj_backend_copy.empty())
      items.push_back({"backend", &traj_backend_copy});
    for (const auto& it : items)
    {
      for (auto t : types)
      {
        StampedPoses aligned;
        AteResult r = computeAte(*it.traj, dataset.groundtruth(), t, &aligned);
        if (!r.valid)
        {
          summary << "  " << it.label << ": not enough poses for evaluation\n";
          break;
        }
        printAte(summary, it.label, r, t);
        if (t == save_type)
          writeTumTrajectory((out_dir / ("stamped_traj_" + it.label + "_aligned_" +
                                         alignmentName(t) + ".txt")).string(), aligned);
      }
    }
    if (mono_vo)
      summary << "  (monocular without the VIO backend has no metric scale: use the sim3 row)\n";
  }
  if (gps_end_time > 0.0 && !dataset.groundtruth().empty())
  {
    // errors WITHOUT any alignment after the GPS hand-off (GPS-denied navigation)
    StampedPoses denied;
    for (const auto& p : traj_frontend)
      if (static_cast<double>(p.timestamp_ns) * 1e-9 > gps_end_time)
        denied.push_back(p);
    StampedPoses dummy, gt_m;
    AteResult r = computeAte(denied, dataset.groundtruth(), AlignmentType::kNone, &dummy, &gt_m);
    summary << "  ------------- GPS-denied part (after t = " << std::fixed << std::setprecision(1)
            << FLAGS_gps_aiding_duration << " s), NO alignment -------------\n";
    if (r.valid)
    {
      // horizontal error and final error
      double sum_h2 = 0.0, max_h = 0.0;
      for (size_t i = 0; i < dummy.size(); ++i)
      {
        const double eh = (dummy[i].p - gt_m[i].p).head<2>().norm();
        sum_h2 += eh * eh;
        max_h = std::max(max_h, eh);
      }
      const Eigen::Vector3d ef = dummy.back().p - gt_m.back().p;
      summary << std::setprecision(1)
              << "  duration " << (dummy.back().timestamp_ns - dummy.front().timestamp_ns) * 1e-9
              << " s, GPS path " << r.trajectory_length << " m\n"
              << "  3D RMSE " << r.rmse << " m, horizontal RMSE "
              << std::sqrt(sum_h2 / static_cast<double>(dummy.size())) << " m, max horizontal "
              << max_h << " m\n"
              << "  final error: horizontal " << ef.head<2>().norm() << " m ("
              << std::setprecision(2) << 100.0 * ef.head<2>().norm() / std::max(1.0, r.trajectory_length)
              << " % of path), vertical " << std::setprecision(1) << ef.z() << " m\n";
    }
    else
    {
      summary << "  not enough poses after the hand-off\n";
    }
  }
  summary << "===============================================================\n";
  std::cout << summary.str() << std::flush;
  {
    std::ofstream f(out_dir / "results.txt");
    f << summary.str();
    params.printUsed(f);
  }
  vis.saveTrajectoryImage((out_dir / "trajectory_top_view.png").string());

  if (FLAGS_viz && FLAGS_wait_at_end && !quit)
  {
    std::cout << "Press any key in an OpenCV window to exit." << std::endl;
    vis.spin(0);
  }

  runner.reset();
  google::ShutdownGoogleLogging();
  return 0;
}
