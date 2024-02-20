#pragma once

#include "agilib/base/parameter_base.hpp"
#include "agilib/math/types.hpp"
#include "agilib/utils/logger.hpp"

namespace agi {

struct BodyDragParameters : public ParameterBase {
  BodyDragParameters();

  using ParameterBase::load;
  bool load(const Yaml& node) override;

  bool valid() const override;

  Scalar cxy_;  // Drag coefficient in body x/y plane
  Scalar cz_;   // Drag coefficient in body z plane
  Scalar ax_;   // Area of the Quadcopter
  Scalar ay_;   // Area of the Quadcopter
  Scalar az_;   // Area of the Quadcopter
  Scalar rho_;  // ISA air density [kg / m^3]
};

}  // namespace agi
