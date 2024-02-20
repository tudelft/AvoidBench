#pragma once

#include <condition_variable>
#include <mutex>
#include <thread>

#include "agilib/base/module.hpp"
#include "agilib/types/command.hpp"
#include "agilib/types/feedback.hpp"
#include "agilib/types/quad_state.hpp"
#include "agilib/types/setpoint.hpp"
#include "agilib/utils/agi_watchdog.hpp"
#include "agilib/utils/logger.hpp"
#include "agilib/utils/median_filter.hpp"
#include "agilib/utils/timer.hpp"

namespace agi {

using FeedbackCallbackFunction = std::function<void(const Feedback&)>;

class BridgeBase : public Module<BridgeBase> {
 public:
  BridgeBase(const std::string& name, const TimeFunction time_function,
             const Scalar timeout = 0.10, const int n_max_timeouts = 10);
  virtual ~BridgeBase();

  virtual bool send(const Command& command) final;
  virtual bool sendAB(const Setpoint& setpoint) final;
  virtual bool activate() final;
  virtual bool deactivate() final;
  virtual void reset();

  virtual bool active() const final;
  virtual bool locked() const final;
  virtual void setVoltage(const Scalar voltage) final;
  virtual Scalar getVoltage() const final;

  virtual bool getFeedback(Feedback* const feedback = nullptr);
  virtual void registerFeedbackCallback(FeedbackCallbackFunction function);

 protected:
  virtual bool sendCommand(const Command& command, const bool active) = 0;
  virtual bool sendReferenceCommand(const Setpoint& setpoint, const bool active) {
    return true;
  }
  virtual void guardTimeout();
  void voltageTimeout() {
    logger_.warn("voltage value not updated for a long time");
  };

  const Scalar timeout_;
  const int n_max_timeouts_;
  const TimeFunction time_function_;

  bool shutdown_{false};
  std::thread timeout_guard_thread_;
  std::mutex timeout_wait_mutex_;
  std::condition_variable timeout_reset_cv_;
  int n_timeouts_{0};
  bool active_ = false;
  bool got_command_ = false;

  Median<Scalar, 15> voltage_{15.5};
  Scalar latest_raw_voltage{15.5};
  std::vector<FeedbackCallbackFunction> feedback_callbacks_;
  AgiWatchdog voltage_watchdog_;
};

}  // namespace agi
