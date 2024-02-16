
#pragma once

#include "agilib/base/parameter_base.hpp"

namespace agi {

struct PidParameters : public ParameterBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PidParameters();

  using ParameterBase::load;
  bool load(const Yaml& node) override;

  Vector<3> kp_rate_;
  Vector<3> kd_rate_;
  Vector<3> ki_rate_;
  Vector<3> integration_max_;
  Scalar filter_sampling_frequency_;
  Scalar filter_cutoff_frequency_;

  bool valid() const override;
};

}  // namespace agi
