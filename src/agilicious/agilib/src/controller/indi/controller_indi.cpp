#include "agilib/controller/indi/controller_indi.hpp"

namespace agi {

IndiController::IndiController(const Quadrotor& quad,
                               const std::shared_ptr<IndiParameters>& params)
  : ControllerBase("INDI"),
    quad_(quad),
    params_(params),
    filterGyr_(params->filter_cutoff_frequency_,
               params->filter_sampling_frequency_, 0.0),
    filterMot_(params->filter_cutoff_frequency_,
               params->filter_sampling_frequency_, 0.0),
    G_inv_(quad_.getAllocationMatrix().inverse()) {}

IndiController::~IndiController() {}

bool IndiController::getCommand(const QuadState& state,
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

  const bool has_imu = imu_.valid();
  logger_.addPublishingVariable("Motors", state.mot);
  logger_.addPublishingVariable("imus", imu_.omega);
  const Vector<3> gyrB = has_imu ? imu_.omega : Vector<3>(state.w);

  filterGyr_.add(gyrB);
  const Vector<3> omega_f_dot = filterGyr_.derivative();

  filterMot_.add(state.mot);
  const Vector<4> wf = filterMot_();
  const Vector<4> thrusts_f = quad_.thrust_map_(0) * wf.asDiagonal() * wf;

  Command command;
  command.t = state.t;
  command.thrusts.setZero();

  const Vector<3> alpha_cmd = references.at(0).state.tau;
  const Vector<3> tau_f = (quad_.getAllocationMatrix() * thrusts_f).tail(3);

  Vector<4> mu;
  mu(0) = references.front().input.collective_thrust * quad_.m_;
  mu.tail(3) = tau_f + quad_.J_ * (alpha_cmd - omega_f_dot);

  Vector<4> mu_ndi;
  mu_ndi(0) = mu(0);
  mu_ndi.tail(3) = quad_.J_ * alpha_cmd + state.w.cross(quad_.J_ * state.w);

  // use ndi calculated tz_cmd to prevent yaw osscilation.
  mu(3) = mu_ndi(3);

  if (!params_->use_indi_) mu = mu_ndi;

  command.thrusts = G_inv_ * mu;
  command.collective_thrust = command.thrusts.sum();
  setpoints->push_back({state, command});

  return true;
}

bool IndiController::updateParameters(
  const Quadrotor& quad, const std::shared_ptr<IndiParameters> params) {
  return updateParameters(quad) && updateParameters(params);
}

bool IndiController::updateParameters(const Quadrotor& quad) {
  if (!quad.valid()) {
    logger_.error("Invalid Quadrotor model passed and not applied!");
    logger_ << quad;
    return false;
  }

  quad_ = quad;

  return true;
}

bool IndiController::updateParameters(
  const std::shared_ptr<IndiParameters> params) {
  if (!params->valid()) {
    logger_.error(
      "Invalid INDI Parameters passed. Will not apply any changes!");
    return false;
  }

  params_ = params;

  return true;
}

std::shared_ptr<IndiParameters> IndiController::getParameters() {
  return params_;
}

}  // namespace agi
