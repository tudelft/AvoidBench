#include "agilib/simulator/low_level_controller_betaflight.hpp"

namespace agi {

LowLevelControllerBetaflight::LowLevelControllerBetaflight(
  Quadrotor quad, const Scalar ctrl_dt,
  const LowLevelControllerBetaflightParams& params, const std::string& name)
  : LowLevelControllerBase(quad, ctrl_dt, name), params_(params) {
  updateQuad(quad);
}


bool LowLevelControllerBetaflight::updateQuad(const Quadrotor& quad) {
  gyro_lpf_ = std::make_shared<LowPassFilter<3>>(params_.freq_lpf_gyro_,
                                                 1. / ctrl_dt_, 0);
  dterm_lpf_ = std::make_shared<LowPassFilter<3>>(params_.freq_lpf_dterm,
                                                  1. / ctrl_dt_, 0);
  error_integral_.setZero();
  quad_ = quad;
  return true;
}


bool LowLevelControllerBetaflight::setCommand(const Command& cmd) {
  if (!cmd.valid()) return false;

  if (cmd.isRatesThrust()) {
    cmd_ = cmd;
    cmd_.collective_thrust =
      quad_.clampCollectiveThrust(cmd_.collective_thrust);
    cmd_.omega = quad_.clampBodyrates(cmd_.omega);
  } else {
    cmd_ = cmd;
    cmd_.thrusts = quad_.clampThrust(cmd.thrusts);
  }

  return true;
}


bool LowLevelControllerBetaflight::setState(const QuadState& state) {
  if (!state.valid()) return false;
  state_ = state;
  return true;
}


void LowLevelControllerBetaflight::run() {
  // If command is invalid, just switch off motors
  if (!cmd_.valid()) {
    motor_omega_des_.setZero();
    return;
  }

  if (!cmd_.isRatesThrust()) {
    Vector<4> motor_thrusts;
    motor_thrusts = cmd_.thrusts;
    motor_thrusts = quad_.clampThrust(motor_thrusts);
    motor_omega_des_ = quad_.motorThrustToOmega(motor_thrusts);
    return;
  }

  const Scalar sbus =
    params_.tmap_.map(cmd_.collective_thrust * quad_.m_, voltage_);
  const Scalar throttle =
    (sbus - params_.SBUS_MIN_VAL) / params_.SBUS_VAL_RANGE;

  // PID Controller
  const ArrayVector<3> omega_des = cmd_.omega;
  const ArrayVector<3> error_proptional = omega_des - state_.w.array();
  error_integral_ += (omega_des - state_.w.array()) * ctrl_dt_;
  error_integral_ = error_integral_.cwiseMax(-params_.i_gain_limit)
                      .cwiseMin(params_.i_gain_limit);
  const ArrayVector<3> error_derivative =
    dterm_lpf_->add(gyro_lpf_->derivative());
  const Vector<3> body_torque_des =
    params_.kp_ * error_proptional * params_.p_gain_scaling +
    params_.ki_ * error_integral_ * params_.i_gain_scaling +
    params_.kd_ * error_derivative * params_.d_gain_scaling;

  const Vector<4> tlmn(throttle, body_torque_des.x(), body_torque_des.y(),
                       body_torque_des.z());
  const Vector<4> motor_throttle = B_allocation_ * tlmn;
  const Vector<4> motor_dshot_command =
    params_.getDSHOT(motor_throttle).cwiseMax(0);

  for (int i = 0; i < 4; ++i) {
    motor_omega_des_(i) =
      params_.omega_offset + params_.omega_volt * voltage_ +
      params_.omega_cmd_lin * motor_dshot_command(i) +
      params_.omega_cmd_sqrt * std::sqrt(motor_dshot_command(i));
  }
  motor_omega_des_ = motor_omega_des_.cwiseMax(quad_.motor_omega_min_)
                       .cwiseMin(quad_.motor_omega_max_);
}

}  // namespace agi
