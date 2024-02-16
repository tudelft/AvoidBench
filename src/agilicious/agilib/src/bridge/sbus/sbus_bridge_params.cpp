#include "agilib/bridge/sbus/sbus_bridge_params.hpp"

#include "agilib/utils/logger.hpp"

namespace agi {

bool SbusParams::load(const fs::path& filename) {
  const Yaml node = Yaml(filename);
  bool check = load(node);
  if (node["thrust_map"].isNull())
    throw ParameterException("No thrust map specified!");
  fs::path thrust_map_file = node["thrust_map"].as<std::string>();
  check &= thrust_map_.load(filename.parent_path() / thrust_map_file);
  return check;
}

bool SbusParams::load(const Yaml& node) {
  serial_settings.port = node["port"].as<std::string>();

  node["timeout"].getIfDefined(timeout);

  node["n_timeouts_for_lock"].getIfDefined(n_timeouts_for_lock);

  if (node["omega_max"].getIfDefined(omega_max)) omega_max *= M_PI / 180.0;

  return !serial_settings.port.empty();
}

std::ostream& operator<<(std::ostream& os, const SbusParams& settings) {
  os << "SBUS Settings:\n" << settings.serial_settings;
  os << "Guard Settings:\n"
     << "timeout:              " << settings.timeout << "s\n"
     << "timeouts before lock: " << settings.n_timeouts_for_lock << std::endl;

  return os;
}

}  // namespace agi
