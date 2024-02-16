#pragma once

#include "agilib/base/parameter_base.hpp"
#include "agilib/math/types.hpp"
#include "agilib/utils/logger.hpp"

namespace agi {

struct LinCubDragParameters : public ParameterBase {
  LinCubDragParameters();

  using ParameterBase::load;
  bool load(const Yaml& node) override;

  bool valid() const override;

  Vector<3> lin_drag_coeff_;
  Vector<3> cub_drag_coeff_;
  Scalar induced_lift_coeff_ = NAN;
};

}  // namespace agi
