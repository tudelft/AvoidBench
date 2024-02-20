#include "agilib/utils/agi_watchdog.hpp"

namespace agi {

AgiWatchdog::AgiWatchdog(std::function<void()> timeout_callback,
                         Scalar timeout_s, bool enabled)
  : timeout_callback_(timeout_callback),
    timeout_s_(timeout_s),
    enabled_(enabled) {
  if (timeout_s_ > 0.0) {
    timeout_thread_ = std::thread(&AgiWatchdog::run, this);
  }
}

AgiWatchdog::~AgiWatchdog() {
  if (timeout_thread_.joinable()) timeout_thread_.join();
}

void AgiWatchdog::run() {
  const std::chrono::milliseconds timeout((int)(timeout_s_ * 1000));
  std::unique_lock<std::mutex> lock(timeout_wait_mutex_);
  while (enabled_) {
    if (timeout_reset_cv_.wait_for(lock, timeout) == std::cv_status::timeout &&
        enabled_) {
      timeout_callback_();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds((int)(1e1)));
  }
}

void AgiWatchdog::refresh() { timeout_reset_cv_.notify_all(); }

}  // namespace agi
