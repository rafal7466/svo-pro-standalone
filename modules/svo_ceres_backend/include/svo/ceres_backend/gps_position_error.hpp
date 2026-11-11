#pragma once
// Standalone addition: absolute position measurement (e.g. GPS converted to the
// local world frame) of the IMU/body origin, as an OKVIS-style error term so it
// can also be handled by the marginalization (EvaluateWithMinimalJacobians).
//   residual = sqrt(information) * (p_meas - p_WS)

#include <ceres/ceres.h>
#include <Eigen/Core>

#include "svo/ceres_backend/error_interface.hpp"
#include "svo/ceres_backend/pose_local_parameterization.hpp"

namespace svo {
namespace ceres_backend {

class GpsPositionError :
    public ceres::SizedCostFunction<3 /* residuals */, 7 /* pose */>,
    public ErrorInterface
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef ceres::SizedCostFunction<3, 7> base_t;
  static const int kNumResiduals = 3;

  GpsPositionError(const Eigen::Vector3d& measurement, double sigma_horizontal,
                   double sigma_vertical)
    : measurement_(measurement)
  {
    sqrt_information_.setZero();
    sqrt_information_(0, 0) = 1.0 / sigma_horizontal;
    sqrt_information_(1, 1) = 1.0 / sigma_horizontal;
    sqrt_information_(2, 2) = 1.0 / sigma_vertical;
  }
  virtual ~GpsPositionError() = default;

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override
  {
    return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
  }

  bool EvaluateWithMinimalJacobians(double const* const* parameters,
                                    double* residuals, double** jacobians,
                                    double** jacobians_minimal) const override
  {
    const Eigen::Vector3d p(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Map<Eigen::Vector3d> r(residuals);
    r = sqrt_information_ * (measurement_ - p);
    if (jacobians != nullptr && jacobians[0] != nullptr)
    {
      Eigen::Matrix<double, 3, 6, Eigen::RowMajor> J_min;
      J_min.setZero();
      J_min.leftCols<3>() = -sqrt_information_;
      Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
      PoseLocalParameterization::liftJacobian(parameters[0], J_lift.data());
      Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> J0(jacobians[0]);
      J0 = J_min * J_lift;
      if (jacobians_minimal != nullptr && jacobians_minimal[0] != nullptr)
      {
        Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> Jm(jacobians_minimal[0]);
        Jm = J_min;
      }
    }
    return true;
  }

  size_t residualDim() const override { return kNumResiduals; }
  size_t parameterBlocks() const override { return base_t::parameter_block_sizes().size(); }
  size_t parameterBlockDim(size_t idx) const override
  {
    return base_t::parameter_block_sizes().at(idx);
  }
  ErrorType typeInfo() const override { return ErrorType::kGpsPositionError; }

 private:
  Eigen::Vector3d measurement_;
  Eigen::Matrix3d sqrt_information_;
};

}  // namespace ceres_backend
}  // namespace svo
