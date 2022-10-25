#include "rpg_common/time_enforcer.h"

#include <chrono>
#include <unistd.h>

namespace rpg_common {

bool TimeEnforcer::waitUntil(const int64_t t_virtual_play_ns)
{
  const int64_t t_wall_now_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::high_resolution_clock::now().time_since_epoch()).count();

  if (!first_call_happened_)
  {
    t_virtual_wall_ns_ = t_wall_now_ns - t_virtual_play_ns;
    first_call_happened_ = true;
    return true;
  }

  const int64_t t_wall_play_ns = t_virtual_play_ns + t_virtual_wall_ns_;
  const int64_t t_now_play_ns = t_wall_play_ns - t_wall_now_ns;

  if (t_now_play_ns > 0)
  {
    usleep(t_now_play_ns / 1e3);
  }

  return t_now_play_ns > 0;
}

}  // namespace rpg_common
