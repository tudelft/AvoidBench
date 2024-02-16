#include <agilib/bridge/ctrl/ctrl_bridge.hpp>

namespace agi {

CtrlBridge::CtrlBridge(const Quadrotor& quad, const CtrlParams& params,
                       const TimeFunction time_function)
  : SerialBridge(params.serial_settings, "CTRL Bridge", time_function,
                 params.timeout, params.n_timeouts_for_lock),
    encoding_(quad, params.thrust_map_, params.ctrl_mode_,
              params.command_throttle_direct, params.thrust_coeff_) {
  if (!serial_port_.addReceiveCallback(std::bind(&CtrlBridge::receiveCallback,
                                                 this, std::placeholders::_1,
                                                 std::placeholders::_2))) {
    logger_.error("Could not bind receive callback to serial port!");
  }
}

bool CtrlBridge::sendCommand(const Command& command, const bool active) {
  Command timed_command = command;
  timed_command.t = time_function_();
  setVoltage(encoding_.getVoltage());
  return sendCommand(timed_command, active, encoding_);
}

void CtrlBridge::receiveCallback(const char* const buffer, const int length) {
  if (encoding_.decodeFeedback(buffer, length))
    for (const FeedbackCallbackFunction& function : feedback_callbacks_)
      function(encoding_.feedback());
}

bool CtrlBridge::getFeedback(Feedback* const feedback) {
  if (feedback == nullptr) return false;

  return encoding_.getFeedback(feedback);
}

}  // namespace agi
