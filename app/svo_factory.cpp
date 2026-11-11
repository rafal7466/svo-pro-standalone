// Standalone port of svo_ros/src/svo_factory.cpp and ceres_backend_factory.cpp
// ros::NodeHandle -> svo::ParamServer (same parameter names and defaults).
#include "svo_factory.h"

#include <svo/svo.h>
#include <svo/common/imu_calibration.h>
#include <svo/frame_handler_mono.h>
#include <svo/frame_handler_stereo.h>
#include <svo/frame_handler_array.h>
#include <svo/imu_handler.h>
#include "param_server.h"

#include <yaml-cpp/yaml.h>

namespace svo {
namespace factory {

BaseOptions loadBaseOptions(const ParamServer& pnh, bool forward_default)
{
  BaseOptions o;
  o.max_n_kfs = vk::param<int>(pnh, "max_n_kfs", 5);
  o.use_imu = vk::param<bool>(pnh, "use_imu", false);
  o.trace_dir = vk::param<std::string>(pnh, "trace_dir", std::string("trace"));
  o.quality_min_fts = vk::param<int>(pnh, "quality_min_fts", 50);
  o.quality_max_fts_drop = vk::param<int>(pnh, "quality_max_drop_fts", 40);
  o.relocalization_max_trials = vk::param<int>(pnh, "relocalization_max_trials", 50);
  o.poseoptim_prior_lambda = vk::param<double>(pnh, "poseoptim_prior_lambda", 0.0);
  o.poseoptim_using_unit_sphere = vk::param<bool>(pnh, "poseoptim_using_unit_sphere", false);
  o.img_align_prior_lambda_rot = vk::param<double>(pnh, "img_align_prior_lambda_rot", 0.0);
  o.img_align_prior_lambda_trans = vk::param<double>(pnh, "img_align_prior_lambda_trans", 0.0);
  o.structure_optimization_max_pts = vk::param<int>(pnh, "structure_optimization_max_pts", 20);
  o.init_map_scale = vk::param<double>(pnh, "map_scale", 1.0);
  std::string default_kf_criterion = forward_default ? "FORWARD" : "DOWNLOOKING";
  if(vk::param<std::string>(pnh, "kfselect_criterion", default_kf_criterion) == "FORWARD")
    o.kfselect_criterion = KeyframeCriterion::FORWARD;
  else
    o.kfselect_criterion = KeyframeCriterion::DOWNLOOKING;
  o.kfselect_min_dist = vk::param<double>(pnh, "kfselect_min_dist", 0.12);
  o.kfselect_numkfs_upper_thresh = vk::param<int>(pnh, "kfselect_numkfs_upper_thresh", 120);
  o.kfselect_numkfs_lower_thresh = vk::param<double>(pnh, "kfselect_numkfs_lower_thresh", 70);
  o.kfselect_min_dist_metric = vk::param<double>(pnh, "kfselect_min_dist_metric", 0.01);
  o.kfselect_min_angle = vk::param<double>(pnh, "kfselect_min_angle", 20);
  o.kfselect_min_disparity = vk::param<double>(pnh, "kfselect_min_disparity", 40);
  o.kfselect_min_num_frames_between_kfs = vk::param<int>(pnh, "kfselect_min_num_frames_between_kfs", 2);
  o.kfselect_backend_max_time_sec = vk::param<double>(pnh, "kfselect_backend_max_time_sec", 3.0);
  o.img_align_max_level = vk::param<int>(pnh, "img_align_max_level", 4);
  o.img_align_min_level = vk::param<int>(pnh, "img_align_min_level", 2);
  o.img_align_robustification = vk::param<bool>(pnh, "img_align_robustification", false);
  o.img_align_use_distortion_jacobian =
      vk::param<bool>(pnh, "img_align_use_distortion_jacobian", false);
  o.img_align_est_illumination_gain =
      vk::param<bool>(pnh, "img_align_est_illumination_gain", false);
  o.img_align_est_illumination_offset =
      vk::param<bool>(pnh, "img_align_est_illumination_offset", false);
  o.poseoptim_thresh = vk::param<double>(pnh, "poseoptim_thresh", 2.0);
  o.update_seeds_with_old_keyframes =
      vk::param<bool>(pnh, "update_seeds_with_old_keyframes", true);
  o.use_async_reprojectors = vk::param<bool>(pnh, "use_async_reprojectors", false);
  o.trace_statistics = vk::param<bool>(pnh, "trace_statistics", false);
  o.backend_scale_stable_thresh =
      vk::param<double>(pnh, "backend_scale_stable_thresh", 0.02);
  o.global_map_lc_timeout_sec_ =
      vk::param<double>(pnh, "global_map_timeout_sec", 2.0);
  return o;
}

DetectorOptions loadDetectorOptions(const ParamServer& pnh)
{
  DetectorOptions o;
  o.cell_size = vk::param<int>(pnh, "grid_size", 35);
  o.max_level = vk::param<int>(pnh, "n_pyr_levels", 3) - 1;
  o.threshold_primary = vk::param<int>(pnh, "detector_threshold_primary", 10);
  o.threshold_secondary = vk::param<int>(pnh, "detector_threshold_secondary", 200);
  o.threshold_shitomasi = vk::param<int>(pnh, "detector_threshold_shitomasi", 100);
  if(vk::param<bool>(pnh, "use_edgelets", true))
    o.detector_type = DetectorType::kFastGrad;
  else
    o.detector_type = DetectorType::kFast;
  return o;
}

DepthFilterOptions loadDepthFilterOptions(const ParamServer& pnh)
{
  DepthFilterOptions o;
  o.max_search_level = vk::param<int>(pnh, "n_pyr_levels", 3) - 1;
  o.use_threaded_depthfilter =
      vk::param<bool>(pnh, "use_threaded_depthfilter", true);
  o.seed_convergence_sigma2_thresh =
      vk::param<double>(pnh, "seed_convergence_sigma2_thresh", 200.0);
  o.mappoint_convergence_sigma2_thresh =
      vk::param<double>(pnh, "mappoint_convergence_sigma2_thresh", 500.0);
  o.scan_epi_unit_sphere = vk::param<bool>(pnh, "scan_epi_unit_sphere", false);
  o.affine_est_offset= vk::param<bool>(pnh, "depth_filter_affine_est_offset", true);
  o.affine_est_gain = vk::param<bool>(pnh, "depth_filter_affine_est_gain", false);
  o.max_n_seeds_per_frame = static_cast<size_t>(
        static_cast<double>(vk::param<int>(pnh, "max_fts", 120))
        * vk::param<double>(pnh, "max_seeds_ratio", 3.0));
  o.max_map_seeds_per_frame = static_cast<size_t>(
        static_cast<double>(vk::param<int>(pnh, "max_map_fts", 120)));
  o.extra_map_points =
      vk::param<bool>(pnh, "depth_filter_extra_map_points", false);
  if(vk::param<bool>(pnh, "runlc", false) && !o.extra_map_points)
  {
    LOG(WARNING) << "Loop closure requires extra map points, "
                 << " but the option is not set, overriding to true.";
    o.extra_map_points = true;
  }
  return o;
}

InitializationOptions loadInitializationOptions(const ParamServer& pnh)
{
  InitializationOptions o;
  o.init_min_features = vk::param<int>(pnh, "init_min_features", 100);
  o.init_min_tracked = vk::param<int>(pnh, "init_min_tracked", 80);
  o.init_min_inliers = vk::param<int>(pnh, "init_min_inliers", 70);
  o.init_min_disparity = vk::param<double>(pnh, "init_min_disparity", 40.0);
  o.init_min_features_factor = vk::param<double>(pnh, "init_min_features_factor", 2.0);
  o.reproj_error_thresh = vk::param<double>(pnh, "reproj_err_thresh", 2.0);
  o.init_disparity_pivot_ratio = vk::param<double>(pnh, "init_disparity_pivot_ratio", 0.5);
  std::string init_method = vk::param<std::string>(pnh, "init_method", "FivePoint");
  if(init_method == "Homography")
    o.init_type = InitializerType::kHomography;
  else if(init_method == "TwoPoint")
    o.init_type = InitializerType::kTwoPoint;
  else if(init_method == "FivePoint")
    o.init_type = InitializerType::kFivePoint;
  else if(init_method == "OneShot")
    o.init_type = InitializerType::kOneShot;
  else
    SVO_ERROR_STREAM("Initialization Method not supported: " << init_method);
  return o;
}

FeatureTrackerOptions loadTrackerOptions(const ParamServer& pnh)
{
  FeatureTrackerOptions o;
  o.klt_max_level = vk::param<int>(pnh, "klt_max_level", 4);
  o.klt_min_level = vk::param<int>(pnh, "klt_min_level", 0.001);
  return o;
}

ReprojectorOptions loadReprojectorOptions(const ParamServer& pnh)
{
  ReprojectorOptions o;
  o.max_n_kfs = vk::param<int>(pnh, "reprojector_max_n_kfs", 5);
  o.max_n_features_per_frame = vk::param<int>(pnh, "max_fts", 160);
  o.cell_size = vk::param<int>(pnh, "grid_size", 35);
  o.reproject_unconverged_seeds =
      vk::param<bool>(pnh, "reproject_unconverged_seeds", true);
  o.max_unconverged_seeds_ratio =
      vk::param<double>(pnh, "max_unconverged_seeds_ratio", -1.0);
  o.min_required_features =
      vk::param<int>(pnh, "quality_min_fts", 50);
  o.seed_sigma2_thresh =
      vk::param<double>(pnh, "seed_convergence_sigma2_thresh", 200.0);

  o.affine_est_offset =
      vk::param<bool>(pnh, "reprojector_affine_est_offset", true);
  o.affine_est_gain =
      vk::param<bool>(pnh, "reprojector_affine_est_gain", false);
  o.max_fixed_landmarks =
      vk::param<int>(pnh, "reprojector_max_fixed_landmarks", 50);
  o.max_n_global_kfs =
      vk::param<int>(pnh, "reprojector_max_n_global_kfs", 20);
  o.use_kfs_from_global_map =
      vk::param<bool>(pnh, "reprojector_use_kfs_from_global_map", false);
  o.fixed_lm_grid_size =
      vk::param<int>(pnh, "reprojector_fixed_lm_grid_size", 50);

  return o;
}

CameraBundle::Ptr loadCameraFromYaml(const ParamServer& pnh)
{
  std::string calib_file = vk::param<std::string>(pnh, "calib_file", "~/cam.yaml");
  CameraBundle::Ptr ncam = CameraBundle::loadFromYaml(calib_file);
  std::cout << "loaded " << ncam->numCameras() << " cameras";
  for(const auto& cam : ncam->getCameraVector())
    cam->printParameters(std::cout, "");
  return ncam;
}





StereoTriangulationOptions loadStereoOptions(const ParamServer& pnh)
{
  StereoTriangulationOptions o;
  o.triangulate_n_features = vk::param<int>(pnh, "max_fts", 120);
  o.max_depth_inv = vk::param<double>(pnh, "max_depth_inv", 1.0/50.0);
  o.min_depth_inv = vk::param<double>(pnh, "min_depth_inv", 1.0/0.5);
  o.mean_depth_inv = vk::param<double>(pnh, "mean_depth_inv", 1.0/2.0);
  return o;
}

ImuHandler::Ptr getImuHandler(const ParamServer& pnh)
{
  std::string calib_file = vk::param<std::string>(pnh, "calib_file", "");
  ImuCalibration imu_calib = ImuHandler::loadCalibrationFromFile(calib_file);
  imu_calib.print("Loaded IMU Calibration");
  ImuInitialization imu_init = ImuHandler::loadInitializationFromFile(calib_file);
  imu_init.print("Loaded IMU Initialization");
  IMUHandlerOptions options;
  options.temporal_stationary_check =
      vk::param<bool>(pnh, "imu_temporal_stationary_check", false);
  options.temporal_window_length_sec_ =
      vk::param<double>(pnh, "imu_temporal_window_length_sec", 0.5);
  options.stationary_acc_sigma_thresh_ =
      vk::param<double>(pnh, "stationary_acc_sigma_thresh", 0.0);
  options.stationary_gyr_sigma_thresh_ =
      vk::param<double>(pnh, "stationary_gyr_sigma_thresh", 0.0);
  ImuHandler::Ptr imu_handler(new ImuHandler(imu_calib, imu_init, options));
  return imu_handler;
}

void setInitialPose(const ParamServer& pnh, FrameHandlerBase& vo)
{
  Transformation T_world_imuinit(
        Quaternion(vk::param<double>(pnh, "T_world_imuinit/qw", 1.0),
                   vk::param<double>(pnh, "T_world_imuinit/qx", 0.0),
                   vk::param<double>(pnh, "T_world_imuinit/qy", 0.0),
                   vk::param<double>(pnh, "T_world_imuinit/qz", 0.0)),
        Vector3d(vk::param<double>(pnh, "T_world_imuinit/tx", 0.0),
                 vk::param<double>(pnh, "T_world_imuinit/ty", 0.0),
                 vk::param<double>(pnh, "T_world_imuinit/tz", 0.0)));
  vo.setInitialImuPose(T_world_imuinit);
}


FrameHandlerMono::Ptr makeMono(const ParamServer& pnh, const CameraBundlePtr& cam)
{
  // Create camera
  CameraBundle::Ptr ncam = (cam) ? cam : loadCameraFromYaml(pnh);
  if (ncam->numCameras() > 1)
  {
    LOG(WARNING) << "Load more cameras than needed, will erase from the end.";
    ncam->keepFirstNCams(1);
  }

  // Init VO
  FrameHandlerMono::Ptr vo =
      std::make_shared<FrameHandlerMono>(
        loadBaseOptions(pnh, false),
        loadDepthFilterOptions(pnh),
        loadDetectorOptions(pnh),
        loadInitializationOptions(pnh),
        loadReprojectorOptions(pnh),
        loadTrackerOptions(pnh),
        ncam);

  // Get initial position and orientation of IMU
  setInitialPose(pnh, *vo);

  return vo;
}

FrameHandlerStereo::Ptr makeStereo(const ParamServer& pnh, const CameraBundlePtr& cam)
{
  // Load cameras
  CameraBundle::Ptr ncam = (cam) ? cam : loadCameraFromYaml(pnh);
  if (ncam->numCameras() > 2)
  {
    LOG(WARNING) << "Load more cameras than needed, will erase from the end.";
    ncam->keepFirstNCams(2);
  }

  // Init VO
  InitializationOptions init_options = loadInitializationOptions(pnh);
  init_options.init_type = InitializerType::kStereo;
  FrameHandlerStereo::Ptr vo =
      std::make_shared<FrameHandlerStereo>(
        loadBaseOptions(pnh, true),
        loadDepthFilterOptions(pnh),
        loadDetectorOptions(pnh),
        init_options,
        loadStereoOptions(pnh),
        loadReprojectorOptions(pnh),
        loadTrackerOptions(pnh),
        ncam);

  // Get initial position and orientation of IMU
  setInitialPose(pnh, *vo);

  return vo;
}

FrameHandlerArray::Ptr makeArray(const ParamServer& pnh, const CameraBundlePtr& cam)
{
  // Load cameras
  CameraBundle::Ptr ncam = (cam) ? cam : loadCameraFromYaml(pnh);

  // Init VO
  InitializationOptions init_options = loadInitializationOptions(pnh);
  init_options.init_type = InitializerType::kArrayGeometric;
  init_options.init_min_disparity = 25;
  DepthFilterOptions depth_filter_options = loadDepthFilterOptions(pnh);
  depth_filter_options.verbose = true;
  FrameHandlerArray::Ptr vo =
      std::make_shared<FrameHandlerArray>(
        loadBaseOptions(pnh, true),
        depth_filter_options,
        loadDetectorOptions(pnh),
        init_options,
        loadReprojectorOptions(pnh),
        loadTrackerOptions(pnh),
        ncam);

  // Get initial position and orientation of IMU
  setInitialPose(pnh, *vo);

  return vo;
}

} // namespace factory
} // namespace svo

// ---------------------------------------------------------------------------
// Ceres backend factory (from svo_ros/src/ceres_backend_factory.cpp)
// ---------------------------------------------------------------------------
#include <svo/ceres_backend_interface.hpp>
#include <svo/motion_detector.hpp>

namespace svo {
namespace ceres_backend_factory {

MotionDetectorOptions loadMotionDetectorOptions(const ParamServer& pnh)
{
  MotionDetectorOptions o;
  o.px_diff_threshold =
      vk::param<double>(pnh, "zero_motion_px_diff_threshold", 0.5);
  o.ratio_moving_pixels_threshold =
      vk::param<double>(pnh, "zero_motion_ratio_moving_pixels_threshold", 0.1);
  o.min_number_correspondences =
      vk::param<int>(pnh, "zero_motion_min_number_correspondences", 5);
  o.max_features_to_check =
      vk::param<int>(pnh, "zero_motion_max_features_to_check", 100);
  DEBUG_CHECK(o.max_features_to_check>0) << "max_features_to_check must be > 0";
  o.sigma = vk::param<double>(pnh, "zero_motion_sigma", 0.05);
  return o;
}

CeresBackendInterface::Ptr makeBackend(const ParamServer& pnh,
                                       const CameraBundlePtr& camera_bundle)
{
  VLOG(1) << "Initialize Backend.";
  CeresBackendOptions backend_options;
  /// Output ceres optimization progress
  backend_options.verbose =
      vk::param<bool>(pnh, "ceres_verbose", false);
  /// Apply marginalization?
  backend_options.marginalize =
      vk::param<bool>(pnh, "ceres_marginalize", true);
  /// Maximum number of iterations
  backend_options.num_iterations =
      static_cast<size_t>(vk::param<int>(pnh, "ceres_num_iterations", 3));
  backend_options.num_threads =
      static_cast<size_t>(vk::param<int>(pnh, "ceres_num_threads", 2));
  /// Sliding window size of ceres backend
  backend_options.num_imu_frames =
      static_cast<size_t>(vk::param<int>(pnh, "ceres_num_imu_frames", 3));
  /// Number of keyframes of ceres backend
  backend_options.num_keyframes =
      static_cast<size_t>(vk::param<int>(pnh, "max_n_kfs", 5));
  /// Maximum time used to optimize [s]. Set negative to always do the
  /// maximum number of iterations.
  backend_options.max_iteration_time =
      static_cast<double>(
        vk::param<double>(pnh, "ceres_max_iteration_time", -1.0));
  /// Should marginalization term be discarded after loop? Experimental
  backend_options.remove_marginalization_term_after_correction_ = vk::param<bool>(
        pnh, "ceres_remove_marginalization_term_after_loop", false);
  backend_options.recalculate_imu_terms_after_loop = vk::param<bool>(
        pnh, "ceres_recalculate_imu_terms_after_loop", false);
  backend_options.remove_fixation_min_num_fixed_landmarks_ = static_cast<size_t>
      (vk::param<int>(pnh, "ceres_remove_fixation_min_num_fixed_points", 10u));
  backend_options.max_fixed_lm_in_ceres_  = static_cast<size_t>
      (vk::param<int>(pnh, "ceres_max_fixed_landmarks", 50u));

  CeresBackendInterfaceOptions ba_interface_options;
  ba_interface_options.min_num_obs =
      static_cast<size_t>(vk::param<int>(pnh, "backend_min_num_obs", 2));
  ba_interface_options.min_parallax_thresh =
      vk::param<double>(pnh, "backend_min_parallax_deg", 2.0)/180.0*M_PI;
  ba_interface_options.only_use_corners =
      vk::param<bool>(pnh, "backend_only_use_corners", false);
  ba_interface_options.use_zero_motion_detection =
      vk::param<bool>(pnh, "backend_use_zero_motion_detection", true);
  ba_interface_options.backend_zero_motion_check_n_frames = static_cast<int>(
        vk::param<int>(pnh, "backend_zero_motion_check_n_frames", 5));
  ba_interface_options.use_outlier_rejection =
      vk::param<bool>(pnh, "use_outlier_rejection", true);
  ba_interface_options.outlier_rejection_px_threshold =
      vk::param<double>(pnh, "outlier_rejection_px_threshold", 2.0);
  // EXPERIMENTAL: when tracking is bad
  ba_interface_options.skip_optimization_when_tracking_bad =
      vk::param<bool>(pnh, "skip_optimization_when_tracking_bad", false);
  ba_interface_options.min_added_measurements=
      vk::param<int>(pnh, "skip_optim_min_obs", 20);

  // extrinsics refinement
  ba_interface_options.refine_extrinsics =
        vk::param<bool>(pnh, "backend_refine_extrinsics", false);
  ba_interface_options.extrinsics_pos_sigma_meter =
        vk::param<double>(pnh, "backend_extrinsics_pos_sigma_meter", 0.05);
  ba_interface_options.extrinsics_rot_sigma_rad =
        vk::param<double>(pnh, "backend_extrinsics_rot_sigma_deg", 5.0) / 180.0 * M_PI;

  CeresBackendInterface::Ptr ba_interface =
      std::make_shared<CeresBackendInterface>(ba_interface_options,
                                              backend_options,
                                              loadMotionDetectorOptions(pnh),
                                              camera_bundle);

  if(vk::param<bool>(pnh, "ba_parallelized", true))
    ba_interface->startThread();

  return ba_interface;
}

} // namespace ceres_backend_factory
} // namespace svo

