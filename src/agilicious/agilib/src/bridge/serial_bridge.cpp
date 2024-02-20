#include "agilib/bridge/serial_bridge.hpp"

namespace agi {
SerialBridge::SerialBridge(const SerialSettings& serial_settings,
                           const std ::string& name,
                           const TimeFunction time_function,
                           const Scalar timeout, const int n_max_timeouts)
  : BridgeBase(name, time_function, timeout, n_max_timeouts),
    serial_settings_(serial_settings),
    serial_port_(serial_settings_) {}

bool SerialBridge::sendCommand(const Command& command, const bool active,
                               const MsgEncoding& encoding) {
  if (!command.valid()) {
    logger_.error("Command invalid.\n");
    return false;
  }

  char buffer[BUFFER_SIZE];
  int length = BUFFER_SIZE;

  if (!encoding.encode(command, active, buffer, &length)) {
    logger_.error("Encoding is not correct.\n");
    return false;
  }

  int sent_length = serial_port_.send(buffer, length);

  return sent_length >= length;
}

}  // namespace agi
