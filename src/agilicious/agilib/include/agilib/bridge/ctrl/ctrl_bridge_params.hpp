#pragma once

#include "agilib/base/parameter_base.hpp"
#include "agilib/bridge/ctrl/ctrl_msgs_defs.hpp"
#include "agilib/bridge/thrust_map.hpp"
#include "agilib/serial/serial_settings.hpp"

namespace agi {

struct CtrlParams : public ParameterBase {
  bool load(const fs::path& filename) override;
  bool load(const Yaml& node) override;

  friend std::ostream& operator<<(std::ostream& os, const CtrlParams& settings);

  static constexpr char DELIMITER = ctrl::SERIAL_CONTAINER_DELIMITER;

  SerialSettings serial_settings{"/dev/ttyTHS2", 500000, DELIMITER, DELIMITER};

  Scalar timeout{0.05};
  int n_timeouts_for_lock{4};

  ctrl::CTRLMODE ctrl_mode_{ctrl::CTRLMODE::ROTOR_THROTTLE};
  bool command_throttle_direct{false};

  ThrustMap thrust_map_;
  Scalar thrust_coeff_;
};

}  // namespace agi
