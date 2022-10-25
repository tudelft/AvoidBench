#pragma once

#include <thread>

#include "rpg_common/threadsafe_queue.h"
#include "rpg_common/time_enforcer.h"

namespace rpg_common {

template <typename DataType>
class FrequencyWorker
{
 public:
  FrequencyWorker(const double frequency) :
    thread_(&FrequencyWorker<DataType>::workLoop, this),
    frequency_(frequency), period_ns_(1e9 / frequency) {}

  virtual ~FrequencyWorker()
  {
    shutdown();
  }

  void addTask(const DataType& item)
  {
    queue_.push(item);
  }

  void shutdown()
  {
    if (!thread_.joinable())
    {
      LOG(WARNING) << "Redundant shutdown call of real-time worker!";
      return;
    }
    queue_.shutdown();
    thread_.join();
  }

 private:
  virtual void process(const DataType& item) = 0;

  void workLoop()
  {
    TimeEnforcer time_enforcer;
    DataType item;
    for (int64_t loop_time_ns = 0; queue_.skipToLatest(&item);
        loop_time_ns += period_ns_)
    {
      if (!time_enforcer.waitUntil(loop_time_ns))
      {
        LOG(WARNING) << "Failed to enforce frequency " << frequency_;
      }
      process(item);
    }
  }

  ThreadSafeQueue<DataType> queue_;
  std::thread thread_;
  double frequency_;
  int64_t period_ns_;
};

}  // namespace rpg_common
namespace rpg = rpg_common;
