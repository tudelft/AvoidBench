#pragma once

#include "agilib/guard/guard_base.hpp"
#include "agilib/guard/position_guard_params.hpp"
#include "agilib/types/quad_state.hpp"

namespace agi {

class PositionGuard : public GuardBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PositionGuard(const PositionGuardParams& params);
  bool evaluateState(const QuadState& state) override;
  bool update(const QuadState& state) override;
  void reset() override;

 private:
  PositionGuardParams params_;
};
}  // namespace agi
