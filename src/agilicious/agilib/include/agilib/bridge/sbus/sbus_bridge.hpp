#pragma once

#include "agilib/bridge/sbus/sbus_bridge_params.hpp"
#include "agilib/bridge/sbus/sbus_encoding.hpp"
#include "agilib/bridge/serial_bridge.hpp"

namespace agi {
class SbusBridge : public SerialBridge {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SbusBridge(const Quadrotor& quad, const SbusParams& params,
             const TimeFunction time_function);

 protected:
  using SerialBridge::sendCommand;
  virtual bool sendCommand(const Command& command, const bool active) override;

  Quadrotor quad_;
  SbusEncoding encoding_;
};

}  // namespace  agi
