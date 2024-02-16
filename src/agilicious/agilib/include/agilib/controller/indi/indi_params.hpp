#pragma once

#include "agilib/base/parameter_base.hpp"

namespace agi {

struct IndiParameters : public ParameterBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  IndiParameters();

  using ParameterBase::load;
  bool load(const Yaml& node) override;

  Vector<3> kp_rate_;

  Scalar filter_sampling_frequency_;
  Scalar filter_cutoff_frequency_;

  bool use_indi_;

  bool valid() const override;

  friend std::ostream& operator<<(std::ostream& os,
                                  const IndiParameters& params);
};

}  // namespace agi
