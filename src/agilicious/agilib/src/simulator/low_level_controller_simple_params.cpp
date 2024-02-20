#include "agilib/simulator/low_level_controller_simple_params.hpp"

namespace agi {

LowLevelControllerSimpleParams::LowLevelControllerSimpleParams()
  : Kinv_ang_vel_tau(Vector<3>(20.0, 20.0, 40.0).asDiagonal()) {}

bool LowLevelControllerSimpleParams::load(const Yaml& node) {
  if (node.isNull()) return false;

  Kinv_ang_vel_tau =
    Vector<3>(node["p_roll"].as<Scalar>(), node["p_pitch"].as<Scalar>(),
              node["p_yaw"].as<Scalar>())
      .asDiagonal();

  return valid();
}

bool LowLevelControllerSimpleParams::valid() const {
  bool check = true;

  check &= Kinv_ang_vel_tau.allFinite();

  return check;
}

}  // namespace agi
