#include "agilib/guard/position_guard.hpp"

#include "agilib/utils/throttler.hpp"

namespace agi {

PositionGuard::PositionGuard(const PositionGuardParams &params)
  : params_(params) {}

bool PositionGuard::evaluateState(const QuadState &state) {
  if (!state.valid()) {
    return false;
  }

  const bool should_trigger =
    externally_triggered_ ||
    (state.p.array() < params_.pos_lower_bound_.array()).any() ||
    (state.p.array() > params_.pos_upper_bound_.array()).any();

  return should_trigger;
}

bool PositionGuard::update(const QuadState &state) {
  if (!state.valid()) {
    return false;
  }

  const bool should_trigger = evaluateState(state);

  switch (guard_state_) {
    case GuardState::NOT_INIT:
      if (!should_trigger) {
        guard_state_ = GuardState::ACTIVE;
      } else if (externally_triggered_) {
        static Throttler throttler(logger_, 1.0);
        throttler(&Logger::error, "Guard not initialized!");
      }
      break;
    case GuardState::ACTIVE:
      if (should_trigger) {
        guard_state_ = GuardState::TRIGGERED;
      }
      break;
    case GuardState::TRIGGERED:
      // Do nothing, stuck here until reset
      break;
    default:
      break;
  }

  return guard_state_ == GuardState::TRIGGERED;
}

void PositionGuard::reset() {
  guard_state_ = GuardState::NOT_INIT;
  externally_triggered_ = false;
}

}  // namespace agi
