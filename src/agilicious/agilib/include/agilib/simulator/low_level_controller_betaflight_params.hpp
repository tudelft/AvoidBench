#pragma once

#include "agilib/base/parameter_base.hpp"
#include "agilib/bridge/thrust_map.hpp"
#include "agilib/math/types.hpp"
#include "agilib/simulator/low_level_controller_base.hpp"
#include "agilib/utils/logger.hpp"

namespace agi {

struct LowLevelControllerBetaflightParams : public ParameterBase {
  LowLevelControllerBetaflightParams();

  using ParameterBase::load;
  bool load(const Yaml& node) override;
  bool loadThrustMap(const fs::path& agi_param_dir);

  bool valid() const override;

  Vector<4> getDSHOT(const Vector<4>& thr);
  friend std::ostream& operator<<(std::ostream& os,
                                  const LowLevelControllerBetaflightParams& p);

  // Thrust Map Commanded Thrust to SBUS
  fs::path tmap_file_;
  ThrustMap tmap_;

  // Directly put the values from the BFL GUI there
  // PID Parameters
  ArrayVector<3> kp_{40, 40, 30};
  ArrayVector<3> ki_{80, 80, 60};
  ArrayVector<3> kd_{20, 20, 0};

  // Directly put the values from the BFL GUI there
  // Throttle settings
  Scalar bfl_min_throttle_ = 1070;
  Scalar bfl_max_throttle_ = 2000;
  Scalar bfl_dshot_offset_ = 0.055;

  // Directly put the values from the BFL GUI there
  // Lowpass Filter Settings
  Scalar freq_lpf_gyro_ = 250;
  Scalar freq_lpf_dterm = 170;

  // Some properties of the protocols involved
  // DSHOT Command properties
  static constexpr size_t PWM_MAX_VAL = 2000;
  static constexpr size_t PWM_MIN_VAL = 1000;
  static constexpr size_t PWM_RANGE = PWM_MAX_VAL - PWM_MIN_VAL;

  // DSHOT Command properties
  static constexpr size_t DSHOT_MAX_VAL = 2048;
  static constexpr size_t DSHOT_MIN_VAL = 48;
  static constexpr size_t DSHOT_RANGE = DSHOT_MAX_VAL - DSHOT_MIN_VAL;

  // SBUS to normalized motor command
  static constexpr size_t SBUS_MIN_VAL = 192;
  static constexpr size_t SBUS_MAX_VAL = 1792;
  static constexpr size_t SBUS_VAL_RANGE = SBUS_MAX_VAL - SBUS_MIN_VAL;

  // Identified via "BetaflightID_v2.m" (ask Leonard if you want to know more)
  // No reason behind this, it is what it is!
  static constexpr Scalar p_gain_scaling = 1.818e-3;
  static constexpr Scalar i_gain_scaling = 16.67e-6;
  static constexpr Scalar d_gain_scaling = -31.25e-6;
  static constexpr Scalar i_gain_limit = 0.1;

  static constexpr Scalar omega_cmd_sqrt = 79.79;
  static constexpr Scalar omega_cmd_lin = 0.04071;
  static constexpr Scalar omega_volt = 61.31;
  static constexpr Scalar omega_offset = -1933.2;

  Logger logger_{"BetaFlightParams"};
};

}  // namespace agi
