#include "agilib/controller/mpc/controller_mpc.hpp"

namespace agi {

MpcController::MpcController(const Quadrotor& quad,
                             const std::shared_ptr<MpcParameters>& params,
                             const Scalar exec_dt)
  : ControllerBase("MPC", exec_dt),
    quad_(quad),
    params_(params),
    cog_filter_(
      quad_,
      (Vector<4>() << params_->Q_cog_omega_init_, params_->Q_cog_lengths_init_)
        .finished()
        .asDiagonal(),
      (Vector<4>() << params_->Q_cog_omega_, params_->Q_cog_lengths_)
        .finished()
        .asDiagonal(),
      params_->R_cog_.asDiagonal()) {
  updateParameters(quad_, params_);

  pred_dt_ = wrapper_.getDt();
  horizon_length_ = Wrapper::N + 1;
  logger_.advertisePublishingVariable("timing_update_ms");

  // Reset controller
  reset();
}


bool MpcController::getCommand(const QuadState& state,
                               const SetpointVector& reference,
                               SetpointVector* const setpoints) {
  if (setpoints == nullptr) return false;
  timing_update_.tic();

  if (!state.valid() || reference.empty() || !reference.front().input.valid()) {
    logger_.error("Control inputs are not valid!");
    logger_.error("State is valid: [%d]!", state.valid());
    logger_.error("References are empty: [%d]!", reference.empty());
    logger_.error("Reference is valid: [%d]!", reference.front().input.valid());
    logger_ << reference.front().input;
    return false;
  }

  QuadState limited_state = state;
  limited_state.w = quad_.clampBodyrates(state.w);


  Matrix<Wrapper::NX, Wrapper::N + 1> reference_states =
    Matrix<Wrapper::NX, Wrapper::N + 1>::Zero();
  Matrix<Wrapper::NU, Wrapper::N> reference_inputs =
    hover_input_.replicate<1, Wrapper::N>();

  const int last_setpoint_idx = reference.size() - 1;
  for (int i = 0; i < Wrapper::N + 1; ++i) {
    const Setpoint& setpoint = reference[std::min(i, last_setpoint_idx)];
    reference_states.col(i) = setpoint.state.x.head<Wrapper::NX>();
    if (i < Wrapper::N) {
      if (setpoint.input.isSingleRotorThrusts()) {
        reference_inputs.col(i) = setpoint.input.thrusts;
      } else if (setpoint.input.isRatesThrust()) {
        reference_inputs.col(i) =
          setpoint.input.collective_thrust *
          Vector<Wrapper::NU>::Constant(quad_.m_ / (Scalar)Wrapper::NU);
        reference_states.block<3, 1>(Wrapper::STATEOME, i) =
          setpoint.input.omega;
      } else if (setpoint.state.a.allFinite()) {
        reference_inputs.col(i) =
          Vector<Wrapper::NU>::Constant(setpoint.state.a.norm() / Wrapper::NU);
      }
    }
  }

  wrapper_.setReferences(reference_states.block<Wrapper::NX, Wrapper::N>(0, 0),
                         reference_inputs.block<Wrapper::NU, Wrapper::N>(0, 0));
  wrapper_.setReferenceN(reference_states.col(Wrapper::N));

  timing_solver_.tic();
  const int status_acados =
    wrapper_.update(limited_state.x.head<Wrapper::NX>());
  timing_solver_.toc();
  if (status_acados != acados::ACADOS_SUCCESS) {
    logger_.error("Acados error solving!");
    timing_update_.toc();
    return false;
  }


  setpoints->clear();
  setpoints->reserve(Wrapper::N + 1);
  for (int i = 0; i < Wrapper::N + 1; ++i) {
    Setpoint setpoint;
    setpoint.state.t = state.t + i * pred_dt_;
    setpoint.state.p =
      wrapper_.getState(i).segment<3>(Wrapper::STATEIDX::STATEPOS);
    setpoint.state.qx(0) =
      wrapper_.getState(i)(Wrapper::STATEIDX::STATEATT + 0);
    setpoint.state.qx(1) =
      wrapper_.getState(i)(Wrapper::STATEIDX::STATEATT + 1);
    setpoint.state.qx(2) =
      wrapper_.getState(i)(Wrapper::STATEIDX::STATEATT + 2);
    setpoint.state.qx(3) =
      wrapper_.getState(i)(Wrapper::STATEIDX::STATEATT + 3);
    setpoint.state.w =
      wrapper_.getState(i).segment<3>(Wrapper::STATEIDX::STATEOME);
    setpoint.state.v =
      wrapper_.getState(i).segment<3>(Wrapper::STATEIDX::STATEVEL);
    if (i < Wrapper::N) {
      setpoint.input.t = setpoint.state.t;
      setpoint.input.thrusts = wrapper_.getInput(i);
      setpoint.input.collective_thrust =
        setpoint.input.thrusts.sum() / quad_.m_;
      setpoint.input.omega = setpoint.state.w;
    }

    setpoint.state.tau =
      quad_.J_inv_ *
      (-setpoint.state.w.cross(quad_.J_ * setpoint.state.w) +
       (quad_.getAllocationMatrix() * setpoint.input.thrusts).tail(3));

    setpoints->push_back(setpoint);
  }

  setpoints->at(0).input.omega = setpoints->at(1).input.omega;

  timing_update_.toc();

  if (params_->cog_enable_) {
    cog_filter_.addCommand(setpoints->front().input);
    if (state.p.z() > params_->cog_height_limit_ &&
        state.v.norm() < params_->cog_abs_velocity_limit_ &&
        state.w.norm() < params_->cog_abs_omega_limit_ &&
        ((setpoints->front().input.thrusts - hover_input_).array().abs() /
           hover_input_.array() <
         params_->cog_rel_hover_thrust_limit_)
          .all()) {
      if (cog_filter_.update()) {
        quad_.t_BM_ = cog_filter_.getAbsolutLengths();
        updateParameters(quad_);
      } else {
        logger_.warn("Could not udpate CoG filter!");
      }
    }
  }

  return true;
}

bool MpcController::reset() { return reset(hover_state_); }

bool MpcController::reset(const QuadState& state) {
  if (!state.valid()) return false;
  return reset(state.x.head<Wrapper::NX>());
}

bool MpcController::reset(const Vector<Wrapper::NX>& state) {
  std::lock_guard<std::mutex> guard(acados_mutex_);
  wrapper_.setInitialState(state);
  wrapper_.setReferences(state.replicate(1, Wrapper::N),
                         hover_input_.replicate(1, Wrapper::N));
  wrapper_.setReferenceN(state);
  wrapper_.setStatesPred(state.replicate(1, Wrapper::N + 1));
  wrapper_.setInputsPred(hover_input_.replicate(1, Wrapper::N));

  return wrapper_.update(state) == acados::ACADOS_SUCCESS;
}

void MpcController::addImuSample(const ImuSample& sample) {
  if (params_->cog_enable_) {
    if (!cog_filter_.addImu(sample))
      logger_.warn("Could not add IMU sample to CoG Filter!");
  }
}

void MpcController::logTiming() const {
  logger_.addPublishingVariable(
    "timing_update_ms",
    1000.0 * Vector<3>(timing_update_.mean(), timing_update_.min(),
                       timing_update_.max()));
}

void MpcController::printTiming() const {
  logger_ << timing_update_;
  logger_ << timing_solver_;
}

bool MpcController::updateParameters(
  const Quadrotor& quad, const std::shared_ptr<MpcParameters>& params) {
  return updateParameters(quad) && updateParameters(params);
}

bool MpcController::updateParameters(const Quadrotor& quad) {
  if (!quad.valid()) {
    logger_.error("Invalid Quadrotor model passed and not applied!");
    logger_ << quad;
    return false;
  }

  quad_ = quad;
  hover_input_ = quad_.m_ * hover_input_acc_;

  std::lock_guard<std::mutex> guard(acados_mutex_);
  // Parameters
  wrapper_.setMass(Vector<Wrapper::N + 1>().setConstant(quad_.m_));
  wrapper_.setAeroCoeffs(
    quad_.aero_coeff_1_.replicate(1, Wrapper::N + 1),
    quad_.aero_coeff_3_.replicate(1, Wrapper::N + 1),
    Vector<Wrapper::N + 1>().setConstant(quad_.aero_coeff_h_));
  wrapper_.setArmLengths(quad_.t_BM_);
  wrapper_.setKappa(Vector<Wrapper::N + 1>().setConstant(quad_.kappa_));
  wrapper_.setInertia(quad_.J_.diagonal());

  // Constraints
  wrapper_.setBodyRateConstraints(-quad_.omega_max_, quad_.omega_max_);
  wrapper_.setThrustConstraints(
    Vector<Wrapper::NU>::Constant(quad_.thrust_min_),
    Vector<Wrapper::NU>::Constant(quad_.thrust_max_));


  return true;
}

bool MpcController::updateParameters(
  const std::shared_ptr<MpcParameters>& params) {
  if (!params->valid()) {
    logger_.error("Invalid MPC Parameters passed. Will not apply any changes!");
    return false;
  }

  params_ = params;
  const Matrix<Wrapper::NQ, Wrapper::NQ> Q =
    (Vector<Wrapper::NQ>() << params_->Q_pos_, params_->Q_att_, params_->Q_vel_,
     params_->Q_omega_xy_, params_->Q_omega_z_)
      .finished()
      .asDiagonal();

  const Matrix<Wrapper::NU, Wrapper::NU> R = params_->R_.asDiagonal();
  Matrix<Wrapper::NY, Wrapper::NY> W;
  W.topLeftCorner(Wrapper::NY - Wrapper::NU, Wrapper::NY - Wrapper::NU) = Q;
  W.bottomRightCorner(Wrapper::NU, Wrapper::NU) = R;

  std::lock_guard<std::mutex> guard(acados_mutex_);
  wrapper_.setCosts(W.replicate(1, Wrapper::N));
  wrapper_.setCostN(Q);

  return true;
}

std::shared_ptr<MpcParameters> MpcController::getParameters() {
  return params_;
}

}  // namespace agi
