#pragma once

#include <cstdint>
#include <ctime>

namespace rpg_common
{

// Like Timer, but it measures CPU time (i.e. time minus I/O waits, sleep,
// etc...) of its thread only.
class ThreadTimer
{
public:
  ThreadTimer();
  // Returns duration in seconds.
  double stop();
  void restart();

private:
  int64_t getTimeNs() const;

  clockid_t thread_clock_id_;
  int64_t start_time_ns_;
};

} // namespace rpg_common
namespace rpg = rpg_common;
