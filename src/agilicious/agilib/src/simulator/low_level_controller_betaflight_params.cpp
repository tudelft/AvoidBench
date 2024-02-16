#include "agilib/simulator/low_level_controller_betaflight_params.hpp"

namespace agi {

LowLevelControllerBetaflightParams::LowLevelControllerBetaflightParams() {}

Vector<4> LowLevelControllerBetaflightParams::getDSHOT(const Vector<4>& thr) {
  const Scalar min_throttle = (bfl_min_throttle_ - PWM_MIN_VAL) / PWM_RANGE;
  const Scalar max_throttle = (bfl_max_throttle_ - PWM_MIN_VAL) / PWM_RANGE;
  const Scalar dshot_offset = bfl_dshot_offset_ * DSHOT_RANGE + DSHOT_MIN_VAL;
  const Scalar dshot_slope = DSHOT_MAX_VAL - dshot_offset;

  const ArrayVector<4> throttle =
    thr.cwiseMax(min_throttle).cwiseMin(max_throttle);
  return throttle * dshot_slope + dshot_offset;
}

bool LowLevelControllerBetaflightParams::load(const Yaml& node) {
  if (node.isNull()) return false;

  kp_ << node["PID"]["kp_roll"].as<Scalar>(),
    node["PID"]["kp_pitch"].as<Scalar>(), node["PID"]["kp_yaw"].as<Scalar>();
  ki_ << node["PID"]["ki_roll"].as<Scalar>(),
    node["PID"]["ki_pitch"].as<Scalar>(), node["PID"]["ki_yaw"].as<Scalar>();
  kd_ << node["PID"]["kd_roll"].as<Scalar>(),
    node["PID"]["kd_pitch"].as<Scalar>(), node["PID"]["kd_yaw"].as<Scalar>();
  bfl_min_throttle_ = node["Throttle"]["min_throttle_pwm"].as<Scalar>();
  bfl_max_throttle_ = node["Throttle"]["max_throttle_pwm"].as<Scalar>();
  bfl_dshot_offset_ = node["Throttle"]["dshot_offset"].as<Scalar>();
  freq_lpf_gyro_ = node["Filter"]["gyro_lowpass_freq"].as<Scalar>();
  freq_lpf_dterm = node["Filter"]["dterm_lowpass_freq"].as<Scalar>();

  tmap_file_ = node["thrust_map"].as<std::string>();

  return valid();
}

bool LowLevelControllerBetaflightParams::loadThrustMap(
  const fs::path& agi_param_dir) {
  fs::path thrust_map_file = agi_param_dir / tmap_file_;
  if (!tmap_.load(thrust_map_file)) {
    logger_.error("Failed to load thrust map from [%s].",
                  thrust_map_file.c_str());
    return false;
  }
  logger_.info("Loaded thrust map from [%s].", thrust_map_file.c_str());
  return true;
}

bool LowLevelControllerBetaflightParams::valid() const {
  bool check = true;

  check &= bfl_min_throttle_ >= PWM_MIN_VAL;
  check &= bfl_max_throttle_ <= PWM_MAX_VAL;
  check &= bfl_min_throttle_ < bfl_max_throttle_;
  check &= bfl_dshot_offset_ >= 0;
  check &= bfl_dshot_offset_ <= 1;
  check &= freq_lpf_dterm > 0;
  check &= freq_lpf_gyro_ > 0;

  return check;
}

std::ostream& operator<<(std::ostream& os,
                         const LowLevelControllerBetaflightParams& p) {
  os.precision(3);
  os << std::scientific;
  os << "P Gains        " << p.kp_.transpose() << std::endl
     << "I Gains        " << p.ki_.transpose() << std::endl
     << "D Gains        " << p.kd_.transpose() << std::endl
     << "Throttle Range "
     << "[" << p.bfl_min_throttle_ << " " << p.bfl_max_throttle_ << "]"
     << std::endl
     << "DSHOT Offset   " << p.bfl_dshot_offset_ << std::endl
     << "Gyro LPFs      " << p.freq_lpf_gyro_ << std::endl
     << "DTerm LPF      " << p.freq_lpf_dterm << std::endl;
  os.precision();
  os.unsetf(std::ios::scientific);
  return os;
}

}  // namespace agi
