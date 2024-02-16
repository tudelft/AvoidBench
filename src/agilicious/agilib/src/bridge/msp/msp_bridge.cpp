#include <agilib/bridge/msp/msp_bridge.hpp>

namespace agi {

MspBridge::MspBridge(const Quadrotor& quad, const MspParams& params,
                     const TimeFunction time_function)
  : SerialBridge(params.serial_settings, "MSP Bridge", time_function,
                 params.timeout, params.n_timeouts_for_lock),
    encoding_(params.thrust_map_) {
  if (!serial_port_.addReceiveCallback(std::bind(&MspBridge::receiveCallback,
                                                 this, std::placeholders::_1,
                                                 std::placeholders::_2))) {
    logger_.error("Could not bind receive callback to serial port!");
  }
}

bool MspBridge::sendCommand(const Command& command, const bool active) {
  if (!command.isSingleRotorThrusts()) return false;

  return sendCommand(command, active, encoding_);
}

void MspBridge::receiveCallback(const char* const buffer, const int length) {
  // decode function not finished
}

bool MspBridge::getFeedback(Feedback* const feedback) {
  if (feedback == nullptr) return false;

  *feedback = encoding_.feedback();
  return feedback->valid();
}

}  // namespace agi
