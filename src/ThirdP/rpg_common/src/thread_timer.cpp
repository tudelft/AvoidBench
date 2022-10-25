#include "rpg_common/thread_timer.h"

#include <pthread.h>

namespace rpg_common
{

ThreadTimer::ThreadTimer()
{
  pthread_getcpuclockid(pthread_self(), &thread_clock_id_);
  start_time_ns_ = getTimeNs();
}

double ThreadTimer::stop()
{
  const int64_t stop_time_ns = getTimeNs();
  const int64_t difference_ns = stop_time_ns - start_time_ns_;
  return difference_ns * 1e-9;
}

void ThreadTimer::restart()
{
  start_time_ns_ = getTimeNs();
}

int64_t ThreadTimer::getTimeNs() const
{
  struct timespec time_spec;
  clock_gettime(thread_clock_id_, &time_spec);
  return static_cast<int64_t>(time_spec.tv_sec) * 1000000000 +
      time_spec.tv_nsec;
}

} // namespace rpg_common
