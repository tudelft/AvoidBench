#pragma once

#include "agilib/base/module.hpp"
#include "agilib/types/quad_state.hpp"

namespace agi {

class GuardBase : public Module<GuardBase> {
 public:
  GuardBase(const std::string& name = "Guard Base");
  virtual ~GuardBase() = default;

  virtual bool evaluateState(const QuadState& state) { return false; }
  virtual bool update(const QuadState& state) { return false; };
  virtual void reset() { guard_state_ = GuardState::NOT_INIT; }
  void trigger() { externally_triggered_ = true; }
  bool triggered() const { return guard_state_ == GuardState::TRIGGERED; }
  enum class GuardState { NOT_INIT, ACTIVE, TRIGGERED };

 protected:
  GuardState guard_state_;
  bool externally_triggered_ = false;
};

}  // namespace agi
