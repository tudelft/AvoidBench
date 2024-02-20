#pragma once
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

#include "agilib/math/types.hpp"

namespace agi {

class AgiWatchdog {
 public:
  AgiWatchdog(std::function<void()> timeout_callback, Scalar timeout_s,
              bool enabled = true);
  ~AgiWatchdog();
  bool watchdogTimedOut();
  void refresh();
  inline void enable() { enabled_ = true; };
  inline void disable() { enabled_ = false; };

 private:
  void run();
  std::thread timeout_thread_;
  std::mutex timeout_wait_mutex_;
  std::condition_variable timeout_reset_cv_;
  std::function<void()> timeout_callback_;
  Scalar timeout_s_;
  bool enabled_;
};
}  // namespace agi
