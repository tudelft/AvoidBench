#include "agilib/bridge/sbus/sbus_bridge.hpp"

#include <unistd.h>

#include <chrono>

namespace agi {

SbusBridge::SbusBridge(const Quadrotor& quad, const SbusParams& params,
                       const TimeFunction time_function)
  : SerialBridge(params.serial_settings, "SBUS Bridge", time_function,
                 params.timeout, params.n_timeouts_for_lock),
    quad_{quad},
    encoding_(quad, params.thrust_map_, params.omega_max) {}

bool SbusBridge::sendCommand(const Command& command, const bool active) {
  if (!command.isRatesThrust()) return false;
  encoding_.setVoltage(voltage_());
  return sendCommand(command, active, encoding_);
}

}  // namespace agi
