#pragma once
// Standalone (ROS-free) replacement of the original CeresBackendPublisher.
// Instead of publishing ROS topics it keeps the latest optimized backend state
// and optionally forwards it to a user callback (e.g. trajectory logger).

#include <functional>
#include <memory>
#include <mutex>

#include <svo/common/types.h>
#include <svo/vio_common/backend_types.hpp>

namespace svo {

namespace ceres_backend {
class Map;
}

class CeresBackendPublisher
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<CeresBackendPublisher> Ptr;
  using StateCallback =
      std::function<void(const ViNodeState& state, int64_t timestamp_ns,
                         int32_t bundle_id)>;

  explicit CeresBackendPublisher(
      const std::shared_ptr<ceres_backend::Map>& map_ptr)
    : map_ptr_(map_ptr)
  {}
  ~CeresBackendPublisher() = default;

  Transformation getLastT_W_B() const
  {
    std::lock_guard<std::mutex> lock(mutex_state_);
    return state_.get_T_W_B();
  }

  ViNodeState getLastState() const
  {
    std::lock_guard<std::mutex> lock(mutex_state_);
    return state_;
  }

  void addFrame(const BundleId& bundle_id)
  {
    std::lock_guard<std::mutex> lock(mutex_frame_id_);
    last_added_frame_ = bundle_id;
  }

  void setStateCallback(const StateCallback& cb)
  {
    std::lock_guard<std::mutex> lock(mutex_state_);
    state_callback_ = cb;
  }

  void publish(const ViNodeState& state, const int64_t timestamp,
               const int32_t seq)
  {
    StateCallback cb;
    {
      std::lock_guard<std::mutex> lock(mutex_state_);
      state_ = state;
      state_frame_id_ = seq;
      cb = state_callback_;
    }
    if (cb)
      cb(state, timestamp, seq);
  }

  const std::shared_ptr<ceres_backend::Map>& map() const { return map_ptr_; }

private:
  mutable std::mutex mutex_frame_id_;
  mutable std::mutex mutex_state_;
  std::shared_ptr<ceres_backend::Map> map_ptr_;
  ViNodeState state_;
  BundleId state_frame_id_ = -1;
  BundleId last_added_frame_ = -1;
  StateCallback state_callback_;
};

}  // namespace svo
