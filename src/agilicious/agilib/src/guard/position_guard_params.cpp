#include "agilib/guard/position_guard_params.hpp"

namespace agi {

bool PositionGuardParams::load(const Yaml& node) {
  if (node.isNull()) {
    std::printf("failed to load.\n");
    return false;
  }
  node["pos_lower_bound"] >> pos_lower_bound_;
  node["pos_upper_bound"] >> pos_upper_bound_;

  return valid();
}

bool PositionGuardParams::valid() const {
  bool check = true;
  check &= pos_lower_bound_.allFinite() && pos_upper_bound_.allFinite();

  return check;
}

}  // namespace agi