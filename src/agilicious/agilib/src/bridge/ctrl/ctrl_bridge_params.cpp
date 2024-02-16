#include "agilib/bridge/ctrl/ctrl_bridge_params.hpp"

#include "agilib/utils/logger.hpp"

namespace agi {

bool CtrlParams::load(const fs::path& filename) {
  const Yaml node = Yaml(filename);
  bool check = load(node);
  if (node["thrust_map"].isNull())
    throw ParameterException("No thrust map specified!");
  fs::path thrust_map_file = node["thrust_map"].as<std::string>();
  check &= thrust_map_.load(filename.parent_path() / thrust_map_file);
  return check;
}

bool CtrlParams::load(const Yaml& node) {
  serial_settings.port = node["port"].as<std::string>();
  serial_settings.baudrate = node["baudrate"].as<int>();
  serial_settings.check_multiple_delimiters = true;

  node["timeout"].getIfDefined(timeout);
  node["n_timeouts_for_lock"].getIfDefined(n_timeouts_for_lock);

  if (node["ctrl_mode"].isDefined()) {
    int ctrl_mode = node["ctrl_mode"].as<int>();
    switch (ctrl_mode) {
      case ctrl::CTRLMODE::BODY_RATE:
      case ctrl::CTRLMODE::ROTOR_THROTTLE:
      case ctrl::CTRLMODE::ROTOR_THRUST:
      case ctrl::CTRLMODE::ROTOR_SPEED:
        ctrl_mode_ = (ctrl::CTRLMODE)ctrl_mode;
        break;
      default:
        std::cout << "unknown type of ctrl_mode " << ctrl_mode << std::endl;
        return false;
    }
  }

  node["command_throttle_direct"].getIfDefined(command_throttle_direct);
  node["thrust_coeff"].getIfDefined(thrust_coeff_);

  return !serial_settings.port.empty();
}

std::ostream& operator<<(std::ostream& os, const CtrlParams& settings) {
  os << "CTRL Settings:\n" << settings.serial_settings;
  os << "Guard Settings:\n"
     << "timeout:              " << settings.timeout << "s\n"
     << "timeouts before lock: " << settings.n_timeouts_for_lock << std::endl;

  return os;
}

}  // namespace agi
