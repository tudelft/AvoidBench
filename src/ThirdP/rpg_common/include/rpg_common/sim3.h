#pragma once

#include <kindr/minimal/quat-sim-transform.h>

namespace rpg_common {

typedef kindr::minimal::QuatSimTransform Sim3;

namespace sim3 {

Sim3 mean(const std::vector<Sim3>& sim3s);

}  // namespace sim3

}  // namespace rpg_common
namespace rpg = rpg_common;
