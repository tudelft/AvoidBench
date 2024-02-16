#include "agilib/controller/pid/controller_pid.hpp"

namespace agi {

PidController::PidController(const Quadrotor& quad,
                             const std::shared_ptr<PidParameters>& params)
  : ControllerBase("PID"),
    quad_(quad),
    params_(params),
    filterGyr_(params->filter_cutoff_frequency_,
               params->filter_sampling_frequency_, 0.0),
    t_last_(0.0),
    omega_err_int_(0.0, 0.0, 0.0),
    G_inv_(quad_.getAllocationMatrix().inverse()) {}

PidController::~PidController() {}

bool PidController::getCommand(const QuadState& state,
                               const SetpointVector& references,
                               SetpointVector* const setpoints) {
  if (setpoints == nullptr) return false;
  setpoints->clear();

  if (!state.valid() || references.empty()) {
    logger_.error("Control inputs are not valid!");
    logger_.error("State is valid: [%d]!", state.valid());
    logger_.error("Setpoints are empty: [%d]!", references.empty());
    logger_.error("Setpoint is valid: [%d]!", references.front().input.valid());
    logger_ << references.front().input;
    return false;
  }

  const Vector<3> omega_f = filterGyr_.add(state.w);
  const Vector<3> omega_f_dot = filterGyr_.derivative();

  const Vector<3> omega_err = references.front().input.omega - omega_f;

  if (t_last_ > 0.0) {
    const Scalar dt = state.t - t_last_;
    omega_err_int_ += omega_err * dt;
    omega_err_int_ = omega_err_int_.cwiseMax(-params_->integration_max_)
                       .cwiseMin(params_->integration_max_);
  }
  t_last_ = state.t;


  const Vector<3> alpha_cmd = params_->kp_rate_.cwiseProduct(omega_err) -
                              params_->kd_rate_.cwiseProduct(omega_f_dot) +
                              params_->ki_rate_.cwiseProduct(omega_err_int_);

  Vector<4> mu;
  mu(0) = references.front().input.collective_thrust * quad_.m_;
  mu.tail(3) = quad_.J_ * alpha_cmd + state.w.cross(quad_.J_ * state.w);

  Command command;
  command.t = state.t;
  command.thrusts = G_inv_ * mu;
  command.collective_thrust = command.thrusts.sum();
  setpoints->push_back({state, command});

  return true;
}

bool PidController::updateParameters(
  const Quadrotor& quad, const std::shared_ptr<PidParameters> params) {
  return updateParameters(quad) && updateParameters(params);
}

bool PidController::updateParameters(const Quadrotor& quad) {
  if (!quad.valid()) {
    logger_.error("Invalid Quadrotor model passed and not applied!");
    logger_ << quad;
    return false;
  }

  quad_ = quad;

  return true;
}

bool PidController::updateParameters(
  const std::shared_ptr<PidParameters> params) {
  if (!params->valid()) {
    logger_.error("Invalid PID Parameters passed. Will not apply any changes!");
    return false;
  }

  params_ = params;

  return true;
}

std::shared_ptr<PidParameters> PidController::getParameters() {
  return params_;
}

}  // namespace agi
