#pragma once

#include <memory>

#include "agilib/bridge/bridge_base.hpp"
#include "agilib/bridge/msg_encoding.hpp"
#include "agilib/serial/serial.hpp"
#include "agilib/types/quadrotor.hpp"
#include "agilib/utils/logger.hpp"

namespace agi {

class SerialBridge : public BridgeBase {
 public:
  SerialBridge(const SerialSettings& serial_settings, const std::string& name,
               const TimeFunction time_function, const Scalar timeout = 0.02,
               const int n_max_timeouts = 10);

  bool sendCommand(const Command& command, const bool active,
                   const MsgEncoding& encoding);

  bool isOpen() const { return serial_port_.isOpen(); }

 protected:
  virtual bool sendCommand(const Command& command, const bool active) = 0;

  static constexpr size_t BUFFER_SIZE = SerialPort::BUFFER_SIZE;
  SerialSettings serial_settings_;
  SerialPort serial_port_;
};

}  // namespace agi
