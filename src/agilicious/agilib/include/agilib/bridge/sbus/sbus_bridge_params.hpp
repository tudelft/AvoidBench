#pragma once

#include "agilib/base/parameter_base.hpp"
#include "agilib/bridge/thrust_map.hpp"
#include "agilib/serial/serial_settings.hpp"

namespace agi {

struct SbusParams : public ParameterBase {
  bool load(const fs::path& filename) override;
  bool load(const Yaml& node) override;

  friend std::ostream& operator<<(std::ostream& os, const SbusParams& settings);

  SbusSerialSettings serial_settings{"/dev/ttyTHS2"};

  Scalar timeout{0.05};
  int n_timeouts_for_lock{4};

  Vector<3> omega_max{1.0, 1.0, 1.0};
  ThrustMap thrust_map_;
};

}  // namespace agi
