#pragma once

#include <chrono>

namespace rpg_common {

// Convenience wrapper for std::chrono.
inline int64_t getTimeNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::high_resolution_clock::now().time_since_epoch()).count();
}

}  // namespace rpg_common
namespace rpg = rpg_common;
