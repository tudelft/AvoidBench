#pragma once

#include <stdlib.h>

namespace rpg_common {

// Ensures that time intervals between successive returns from waitUntil()
// have the same length as the difference between the passed virtual times.
class TimeEnforcer
{
public:
  // Returns false if time has already passed.
  bool waitUntil(const int64_t t_virtual_ns);

private:
  bool first_call_happened_ = false;
  int64_t t_virtual_wall_ns_;
};

}  // namespace rpg_common
namespace rpg = rpg_common;
