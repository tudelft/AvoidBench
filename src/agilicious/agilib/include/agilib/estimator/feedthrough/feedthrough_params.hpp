#pragma once

#include "agilib/base/parameter_base.hpp"
#include "agilib/math/types.hpp"
#include "agilib/utils/logger.hpp"

namespace agi {

struct FeedthroughParameters : public ParameterBase {
  FeedthroughParameters();

  using ParameterBase::load;
  bool load(const Yaml& node) override;

  bool valid() const override;

  bool transform_enabled_;
  Scalar roll_;
  Scalar pitch_;
  Scalar yaw_;
  Vector<3> pos_offset_;
};

}  // namespace agi
