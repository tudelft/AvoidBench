#pragma once

#include "agilib/base/parameter_base.hpp"

namespace agi {
struct PositionGuardParams : public ParameterBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using ParameterBase::load;
  bool load(const Yaml& node) override;
  bool valid() const override;

  Vector<3> pos_lower_bound_ =
    Vector<3>::Constant(std::numeric_limits<Scalar>::min());
  Vector<3> pos_upper_bound_ =
    Vector<3>::Constant(std::numeric_limits<Scalar>::max());
};
}  // namespace agi