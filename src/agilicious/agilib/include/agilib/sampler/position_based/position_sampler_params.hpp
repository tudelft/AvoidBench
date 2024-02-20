#pragma once

#include "agilib/base/parameter_base.hpp"

namespace agi {

struct PositionSamplerParameters : public ParameterBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using ParameterBase::load;
  bool load(const Yaml& node) override;

  // step size when searching reference for closest setpoint
  Scalar search_dt_{0.01};
  // Due to imperfect tracking, prioritize future setpoints with this tolerance
  Scalar search_tol_{0.001};
  // use axis-specific weights to compute the distance between setpoints
  Vector<3> axis_weights_sqrt_;

  bool valid() const override;
};

}  // namespace agi
