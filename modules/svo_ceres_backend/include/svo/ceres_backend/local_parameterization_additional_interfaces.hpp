/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *  Copyright (c) 2016, ETH Zurich, Wyss Zurich, Zurich Eye
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Feb 2, 2014
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Zurich Eye
 *********************************************************************************/

/**
 * @file LocalParamizationAdditionalInterfaces.hpp
 * @brief Header file for the LocalParamizationAdditionalInterfaces class.
 * @author Stefan Leutenegger
 */

#pragma once

#include <ceres/manifold.h>

namespace svo {
namespace ceres_backend {

/// \brief Provides some additional interfaces to ceres' LocalParamization
///        than are needed in the generic marginalisation okvisceres::MarginalizationError.
///
/// Standalone port: Ceres >= 2.2 removed ceres::LocalParameterization. This
/// class now derives from ceres::Manifold and maps the legacy (OKVIS style)
/// interface onto it, so the derived parameterisations stay unchanged.
/// NOTE: OKVIS' Minus(x, x_plus_delta, delta) has swapped argument order w.r.t.
/// ceres::Manifold::Minus(y, x, y_minus_x); it is therefore called MinusOkvis.
class LocalParamizationAdditionalInterfaces : public ceres::Manifold
{
 public:
  /// \brief Trivial destructor.
  virtual ~LocalParamizationAdditionalInterfaces() = default;

  // ---- legacy LocalParameterization interface (implemented by subclasses) --
  virtual int GlobalSize() const = 0;
  virtual int LocalSize() const = 0;
  virtual bool ComputeJacobian(const double* x, double* jacobian) const = 0;

  /// \brief Computes the minimal difference between a variable x and a
  ///        perturbed variable x_plus_delta
  virtual bool MinusOkvis(const double* x, const double* x_plus_delta,
                          double* delta) const = 0;

  /// \brief Computes the Jacobian from minimal space to naively
  ///        overparameterised space as used by ceres.
  virtual bool ComputeLiftJacobian(const double* x, double* jacobian) const = 0;

  // ---- ceres::Manifold interface ------------------------------------------
  int AmbientSize() const final { return GlobalSize(); }
  int TangentSize() const final { return LocalSize(); }
  bool PlusJacobian(const double* x, double* jacobian) const final
  {
    return ComputeJacobian(x, jacobian);
  }
  bool Minus(const double* y, const double* x, double* y_minus_x) const final
  {
    return MinusOkvis(x, y, y_minus_x);
  }
  bool MinusJacobian(const double* x, double* jacobian) const final
  {
    return ComputeLiftJacobian(x, jacobian);
  }

  /// \brief Verifies the correctness of an inplementation by means of numeric Jacobians.
  virtual bool verify(const double* x_raw, double purturbation_magnitude = 1.0e-6) const ;
};

}  // namespace ceres_backend
}  // namespace svo

