#include "agilib/bridge/laird/laird_bridge_params.hpp"

#include <cctype>

namespace agi {

LairdParams::LairdParams(const std::string& port, const int baudrate,
                         const SerialMode serial_mode)
  : serial_settings(port, baudrate, serial_mode) {}


bool LairdParams::load(const fs::path& filename) {
  const Yaml node = Yaml(filename);
  return load(node);
}

bool LairdParams::load(const Yaml& node) {
  node["port"].getIfDefined(serial_settings.port);

  node["baudrate"].getIfDefined(serial_settings.baudrate);

  node["single_rotor_thrust"].getIfDefined(single_rotor_thrust);

  if (node["direction"].isDefined()) {
    std::string direction = node["direction"].as<std::string>();
    std::transform(direction.begin(), direction.end(), direction.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    bool receive = false;
    receive |= direction.find("receive") != std::string::npos;
    receive |= direction.find("rx") != std::string::npos;
    serial_settings.serial_mode =
      receive ? SerialMode::Read : SerialMode::Write;
  }

  node["timeout"].getIfDefined(timeout);

  return !serial_settings.port.empty();
}

std::ostream& operator<<(std::ostream& os, const LairdParams& settings) {
  os << "Laird Settings:\n" << settings.serial_settings;
  os << "Guard Settings:\n"
     << "timeout:              " << settings.timeout << "s\n"
     << "timeouts before lock: " << settings.n_timeouts_for_lock << std::endl;

  return os;
}

}  // namespace agi
