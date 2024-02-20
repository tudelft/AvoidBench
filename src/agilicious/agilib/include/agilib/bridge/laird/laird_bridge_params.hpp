#pragma once

#include "agilib/base/parameter_base.hpp"
#include "agilib/serial/serial_settings.hpp"

namespace agi {

struct LairdParams : public ParameterBase {
  LairdParams() = default;
  LairdParams(const std::string& port, const int baudrate,
              const SerialMode serial_mode);

  bool load(const fs::path& filename) override;
  bool load(const Yaml& node) override;

  friend std::ostream& operator<<(std::ostream& os,
                                  const LairdParams& settings);

  SerialSettings serial_settings{"/dev/ttyUSB0", 115200};

  bool single_rotor_thrust{false};
  // Ca use guard to send arm/disarm with zero command, but never lock.
  Scalar timeout{0.0};
  const int n_timeouts_for_lock{0};
};

}  // namespace agi
