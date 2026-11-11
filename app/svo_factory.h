#pragma once
// Standalone port of svo_ros/svo_factory.h and ceres_backend_factory.h
// (ros::NodeHandle replaced by svo::ParamServer).

#include <memory>
#include <svo/common/camera_fwd.h>
#include "param_server.h"

namespace svo {

// forward declarations
class ImuHandler;
class FrameHandlerBase;
class FrameHandlerMono;
class FrameHandlerStereo;
class FrameHandlerArray;
class CeresBackendInterface;

namespace factory {

/// Get IMU Handler.
std::shared_ptr<ImuHandler> getImuHandler(const ParamServer& pnh);

/// Factory for Mono-SVO.
std::shared_ptr<FrameHandlerMono> makeMono(
    const ParamServer& pnh, const CameraBundlePtr& cam = nullptr);

/// Factory for Stereo-SVO.
std::shared_ptr<FrameHandlerStereo> makeStereo(
    const ParamServer& pnh, const CameraBundlePtr& cam = nullptr);

/// Factory for Camera-Array-SVO.
std::shared_ptr<FrameHandlerArray> makeArray(
    const ParamServer& pnh, const CameraBundlePtr& cam = nullptr);

} // namespace factory

namespace ceres_backend_factory {

std::shared_ptr<CeresBackendInterface> makeBackend(
    const ParamServer& pnh, const CameraBundlePtr& camera_bundle);

} // namespace ceres_backend_factory
} // namespace svo
