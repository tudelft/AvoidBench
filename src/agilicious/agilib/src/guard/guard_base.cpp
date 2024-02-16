#include "agilib/guard/guard_base.hpp"

namespace agi {
GuardBase::GuardBase(const std::string& name)
  : Module(name), guard_state_(GuardState::NOT_INIT) {}

}  // namespace agi
