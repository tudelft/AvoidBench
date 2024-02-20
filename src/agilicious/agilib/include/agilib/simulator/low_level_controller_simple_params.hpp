#pragma once

#include "agilib/base/parameter_base.hpp"
#include "agilib/math/types.hpp"
#include "agilib/utils/logger.hpp"

namespace agi {

struct LowLevelControllerSimpleParams : public ParameterBase {
  LowLevelControllerSimpleParams();

  using ParameterBase::load;
  bool load(const Yaml& node) override;

  bool valid() const override;

  // P gain for body rate control
  Matrix<3, 3> Kinv_ang_vel_tau;
};

}  // namespace agi
