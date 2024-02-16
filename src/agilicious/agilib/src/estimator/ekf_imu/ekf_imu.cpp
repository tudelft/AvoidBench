#include "agilib/estimator/ekf_imu/ekf_imu.hpp"

#include <iostream>

#include "agilib/math/gravity.hpp"

namespace agi {

EkfImu::EkfImu(const std::shared_ptr<EkfImuParameters>& params)
  : EstimatorBase("EKF"), imu_last_(NAN, -GVEC, Vector<3>::Zero()) {
  if (!params) {
    params_ = std::make_shared<EkfImuParameters>();
  } else {
    params_ = params;
  }

  if (!updateParameters(params_)) logger_.error("Could not load parameters!");
}


EkfImu::~EkfImu() { printTimings(); }

bool EkfImu::getAt(const Scalar t, QuadState* const state) {
  if (state == nullptr) return false;
  if (std::isnan(t)) return false;

  const Vector<4> motors = motor_speeds_;
  const Vector<4> motor_des = state->motdes;
  state->setZero();
  state->t = t;

  std::lock_guard<std::mutex> lock(mutex_);
  if (params_->update_on_get) process();

  // Catch trivial cases...
  if (t <= t_posterior_) {
    return vectorToState(t_posterior_, posterior_, state);
  }

  bool ret = true;
  ret &= propagatePrior(t);
  ret &= vectorToState(t_prior_, prior_, state);

  if (motors.allFinite()) state->mot = motors;
  if (motor_des.allFinite()) state->motdes = motor_des;

  return ret;
}

bool EkfImu::initialize(const QuadState& state) {
  std::lock_guard<std::mutex> lock(mutex_);
  return init(state);
}

bool EkfImu::init(const QuadState& state) {
  if (!state.valid()) return false;

  if (!poses_.empty()) {
    const auto first_valid_pose = std::lower_bound(
      poses_.begin(), poses_.end(), state.t,
      [](const Pose& pose, const Scalar t) { return pose.t < t; });

    poses_.erase(poses_.begin(), first_valid_pose);
  }

  if (!imus_.empty()) {
    const auto first_valid_imu = std::lower_bound(
      imus_.begin(), imus_.end(), state.t,
      [](const ImuSample& imu, const Scalar t) { return imu.t < t; });

    imus_.erase(imus_.begin(), first_valid_imu);
  }

  stateToVector(state, &t_posterior_, &posterior_);
  t_prior_ = t_posterior_;
  prior_ = posterior_;

  return true;
}

bool EkfImu::addPose(const Pose& pose) {
  if (!pose.valid()) return false;

  std::lock_guard<std::mutex> lock(mutex_);

  if (!std::isfinite(t_posterior_)) {
    QuadState state;
    state.setZero();
    state.t = pose.t;
    state.p = pose.position;
    state.q(pose.attitude);
    return init(state);
  }

  if (pose.t <= t_posterior_) return false;

  poses_.push_back(pose);

  while ((int)poses_.size() > MAX_QUEUE_SIZE) poses_.pop_front();

  process();
  return true;
}

bool EkfImu::addState(const QuadState& state) {
  return addPose({state.t, state.p, state.q()});
}

bool EkfImu::addImu(const ImuSample& imu) {
  if (!imu.valid()) return false;

  std::lock_guard<std::mutex> lock(mutex_);

  if (imu.t < t_posterior_) return false;

  imu_last_ = imu;
  imus_.push_back(imu);
  while ((int)imus_.size() > MAX_QUEUE_SIZE) imus_.pop_front();

  process();
  return true;
}

bool EkfImu::addMotorSpeeds(const Vector<4>& speeds) {
  motor_speeds_ = speeds;
  return true;
}

bool EkfImu::healthy() const {
  return std::isfinite(t_posterior_) && std::isfinite(t_prior_);
}

void EkfImu::logTiming() const {
  logger_.addPublishingVariable(
    "timer_process_ms",
    1000.0 * Vector<3>(timer_process_.mean(), timer_process_.min(),
                       timer_process_.max()));
}

void EkfImu::printTimings(const bool all) const {
  logger_ << timer_process_;
  if (all) {
    logger_ << timer_update_pose_;
    logger_ << timer_compute_gain_;
    logger_ << timer_propagation_;
    logger_ << timer_jacobian_;
  }
}

bool EkfImu::process() {
  if (imus_.empty()) return true;

  ScopedTicToc timer_process_tictoc(timer_process_);

  Scalar t_last_processed_pose = NAN;

  for (const Pose& pose : poses_) {
    if (pose.t < t_posterior_) {
      logger_.warn("Missed Pose update at %1.3g\n", pose.t);
      continue;
    }

    if (pose.t > imus_.back().t) break;

    propagatePriorAndCovariance(pose.t);
    t_posterior_ = t_prior_;
    posterior_ = prior_;
    updatePose(pose);
    t_last_processed_pose = pose.t;

    // Pedantic: exit if queues have been emptied in possible filter re-init.
    if (poses_.empty()) break;
  }

  if (!std::isfinite(t_last_processed_pose)) return true;

  // Remove what was processed
  const auto imu_erase_to_ptr = std::lower_bound(
    imus_.begin(), imus_.end(), t_posterior_,
    [](const ImuSample& imu, const Scalar tt) { return imu.t <= tt; });

  imus_.erase(imus_.begin(), imu_erase_to_ptr);

  const auto pose_erase_to_ptr = std::lower_bound(
    poses_.begin(), poses_.end(), t_last_processed_pose,
    [](const Pose& pose, const Scalar tt) { return pose.t <= tt; });

  poses_.erase(poses_.begin(), pose_erase_to_ptr);

  return true;
}

bool EkfImu::updatePose(const Pose& pose) {
  ScopedTicToc timer_update_tictoc(timer_update_pose_);

  if (!propagatePriorAndCovariance(pose.t)) {
    logger_.error("Could not propagate to %1.6gs", pose.t);
    logger_.error("Prior at %1.6gs", t_prior_);
    logger_.error("Posterior at %1.6gs", t_posterior_);
    logger_.error("IMU queue has %zu samples from %1.6gs to %1.6gs",
                  imus_.size(), imus_.front().t, imus_.back().t);
    logger_.error("Pose queue has %zu samples from %1.6gs to %1.6gs",
                  poses_.size(), poses_.front().t, poses_.back().t);
    return false;
  }

  const Vector<3> p = prior_.segment<IDX::NPOS>(IDX::POS);
  const Quaternion q{prior_(IDX::ATTW), prior_(IDX::ATTX), prior_(IDX::ATTY),
                     prior_(IDX::ATTZ)};

  if (params_->jump_pos_threshold > 0.0 || params_->jump_att_threshold > 0.0) {
    const Vector<3> d_pos = pose.position - p;
    const Quaternion d_q = pose.attitude * q.conjugate();
    const Quaternion d_q_corrected =
      d_q.w() > 0.0 ? d_q : Quaternion(-d_q.w(), -d_q.x(), -d_q.y(), -d_q.z());
    const Scalar d_angle =
      d_q_corrected.w() < 0.99 ? 2.0 * acos(d_q_corrected.w()) : 0.0;
    const bool pos_jump = params_->jump_pos_threshold > 0.0 &&
                          d_pos.norm() > params_->jump_pos_threshold;
    const bool att_jump = params_->jump_att_threshold > 0.0 &&
                          d_angle > params_->jump_att_threshold;
    if (pos_jump || att_jump) {
      logger_.warn(
        "Detected jump in pose measurement!\n"
        "Time: %1.3f\n"
        "Position:  %1.1f m\n"
        "Angle:     %1.1f rad\n",
        pose.t, d_pos.norm(), d_angle);
      if (pos_jump) {
        logger_ << "Prior Pos: " << p.transpose() << std::endl;
        logger_ << "Meas Pos:  " << pose.position.transpose() << std::endl;
      }
      if (att_jump) {
        logger_ << "Prior Att: " << q.coeffs().transpose() << std::endl;
        logger_ << "Meas Att:  " << pose.attitude.coeffs().transpose()
                << std::endl;
      }
      static int n_jumps = 0;
      ++n_jumps;
      if (n_jumps < 10) {
        return true;
      } else {
        n_jumps = 0;
        logger_.warn("Reset state due to too many jumps!");
        QuadState state;
        state.setZero();
        state.t = pose.t;
        state.p = pose.position;
        state.q(pose.attitude);
        return init(state);
      }
    }
  }

  // Pose update residual and jacobian.
  const Vector<SRPOSE> y = (Vector<SRPOSE>() << p - pose.position,
                            (pose.attitude.conjugate() * q).vec())
                             .finished();

  Matrix<SRPOSE, IDX::SIZE> H = Matrix<SRPOSE, IDX::SIZE>::Zero();
  H.block<3, 3>(0, IDX::POS) = Matrix<3, 3>::Identity();
  H.block<3, 4>(3, IDX::ATT) =
    Q_left(pose.attitude.conjugate()).bottomRows<3>();

  // Compute Gain
  ScopedTicToc timer_gain_tictoc(timer_compute_gain_);

  const StateMatrix P = 0.5 * (P_ + P_.transpose());
  const Matrix<SRPOSE, IDX::SIZE> HP = H * P;
  const Matrix<SRPOSE, SRPOSE> S = HP * H.transpose() + R_pose_;
  const Matrix<SRPOSE, SRPOSE> S_inv = S.inverse();

  if (!S_inv.allFinite()) return false;

  const Matrix<IDX::SIZE, SRPOSE> K = P * H.transpose() * S_inv;
  StateVector new_posterior = prior_ - K * y;
  new_posterior.segment<IDX::NATT>(IDX::ATT).normalize();
  const StateMatrix P_new = P - K * HP;

  if (!new_posterior.allFinite()) return false;

  t_posterior_ = t_prior_;
  posterior_ = new_posterior;
  prior_ = new_posterior;
  P_ = 0.5 * (P_new + P_new.transpose());

  return true;
}

bool EkfImu::propagatePrior(const Scalar t) {
  if (std::isnan(t)) return false;

  ScopedTicToc timer_propagation_tictoc(timer_propagation_);

  if (t < t_prior_) {
    if (t < t_posterior_) return false;
    t_prior_ = t_posterior_;
    prior_ = posterior_;
  }

  auto imu_ptr = std::lower_bound(
    imus_.begin(), imus_.end(), t_prior_,
    [](const ImuSample& imu, const Scalar tt) { return imu.t <= tt; });

  Vector<SRIMU> imu_data = Vector<SRIMU>::Zero();
  Scalar t_target = t;

  if (imu_ptr < imus_.end()) {
    t_target = std::min(imu_ptr->t, t);
    imu_data.head(3) = imu_ptr->omega;
    imu_data.tail(3) = imu_ptr->acc;
  } else {
    imu_data.head(3) = imu_last_.omega;
    imu_data.tail(3) = imu_last_.acc;
  }

  static constexpr Scalar dt_max = 1e-4;
  while (t_prior_ < t) {
    if (imu_ptr < imus_.end()) {
      if (imu_ptr->t <= t_prior_) ++imu_ptr;
      if (imu_ptr < imus_.end()) {
        t_target = std::min(imu_ptr->t, t);
        imu_data.head(3) = imu_ptr->omega;
        imu_data.tail(3) = imu_ptr->acc;
      } else {
        t_target = t;
      }
    }

    while (t_prior_ < t_target) {
      const Scalar dt = std::min(t_target - t_prior_, dt_max);

      const Quaternion q{prior_(IDX::ATTW), prior_(IDX::ATTX),
                         prior_(IDX::ATTY), prior_(IDX::ATTZ)};
      const Vector<3> omega =
        imu_data.head(3) - prior_.segment<IDX::NBOME>(IDX::BOME);
      const Vector<3> acc =
        imu_data.tail(3) - prior_.segment<IDX::NBACC>(IDX::BACC);

      prior_.segment<IDX::NPOS>(IDX::POS) +=
        0.5 * dt * prior_.segment<IDX::NVEL>(IDX::VEL);
      prior_.segment<IDX::NVEL>(IDX::VEL) +=
        dt * (GVEC + q.toRotationMatrix() * acc);
      prior_.segment<IDX::NPOS>(IDX::POS) +=
        0.5 * dt * prior_.segment<IDX::NVEL>(IDX::VEL);

      prior_.segment<IDX::NATT>(IDX::ATT) +=
        0.5 * dt * Q_left(q) * Vector<4>(0, omega.x(), omega.y(), omega.z());
      prior_.segment<IDX::NATT>(IDX::ATT).normalize();

      t_prior_ += dt;
      if (dt < dt_max) break;
    }
    t_prior_ = t_target;
  }

  return true;
}

bool EkfImu::propagatePriorAndCovariance(const Scalar t) {
  if (std::isnan(t)) return false;

  ScopedTicToc timer_propagation_tictoc(timer_propagation_);

  if (t < t_prior_) {
    if (t < t_posterior_) return false;
    t_prior_ = t_posterior_;
    prior_ = posterior_;
  }

  auto imu_ptr = std::lower_bound(
    imus_.begin(), imus_.end(), t_prior_,
    [](const ImuSample& imu, const Scalar tt) { return imu.t <= tt; });

  Vector<SRIMU> imu_data = Vector<SRIMU>::Zero();
  Scalar t_target = t;

  if (imu_ptr < imus_.end()) {
    t_target = std::min(imu_ptr->t, t);
    imu_data.head(3) = imu_ptr->omega;
    imu_data.tail(3) = imu_ptr->acc;
  } else {
    imu_data.head(3) = imu_last_.omega;
    imu_data.tail(3) = imu_last_.acc;
  }

  static constexpr Scalar dt_max = 1e-4;

  while (t_prior_ < t) {
    if (imu_ptr < imus_.end()) {
      if (imu_ptr->t <= t_prior_) ++imu_ptr;
      if (imu_ptr < imus_.end()) {
        t_target = std::min(imu_ptr->t, t);
        imu_data.head(3) = imu_ptr->omega;
        imu_data.tail(3) = imu_ptr->acc;
      } else {
        t_target = t;
      }
    }

    while (t_prior_ < t_target) {
      const Scalar dt = std::min(t_target - t_prior_, dt_max);

      const Quaternion q{prior_(IDX::ATTW), prior_(IDX::ATTX),
                         prior_(IDX::ATTY), prior_(IDX::ATTZ)};
      const Vector<3> omega =
        imu_data.head(3) - prior_.segment<IDX::NBOME>(IDX::BOME);
      const Vector<3> acc =
        imu_data.tail(3) - prior_.segment<IDX::NBACC>(IDX::BACC);

      const Matrix<4, 4> Qleftq = Q_left(q);
      const Matrix<3, 3> R = q.toRotationMatrix();

      StateMatrix F = StateMatrix::Zero();
      Matrix<IDX::SIZE, SRIMU> G = Matrix<IDX::SIZE, SRIMU>::Zero();

      // d/dv p_dot
      F.block<IDX::NPOS, IDX::NVEL>(IDX::POS, IDX::VEL) =
        Matrix<3, 3>::Identity();

      // d/dq q_dot
      const Quaternion qomega{0, omega.x(), omega.y(), omega.z()};
      F.block<IDX::NATT, IDX::NATT>(IDX::ATT, IDX::ATT) = 0.5 * Q_right(qomega);

      // d/dbw q_dot
      F.block<IDX::NATT, IDX::NBOME>(IDX::ATT, IDX::BOME) =
        -0.5 * Qleftq.rightCols(3);

      // d/dw q_dot
      G.block<IDX::NATT, 3>(IDX::ATT, 0) = 0.5 * Qleftq.rightCols(3);

      // d/dq v_dot
      const Quaternion qacc{0, acc.x(), acc.y(), acc.z()};
      const Vector<4> v_QrTqa =
        Q_right(q).transpose() * Vector<4>(0, acc.x(), acc.y(), acc.z());
      F.block<IDX::NVEL, IDX::NATT>(IDX::VEL, IDX::ATT) =
        (Q_right(Quaternion(v_QrTqa(0), v_QrTqa(1), v_QrTqa(2), v_QrTqa(3))) +
         Qleftq * Q_left(qacc) * qConjugateJacobian())
          .bottomRows(3);

      // d/dba v_dot
      F.block<IDX::NVEL, IDX::NBACC>(IDX::VEL, IDX::BACC) = -R;

      // d/da v_dot
      G.block<IDX::NVEL, 3>(IDX::VEL, 3) = R;


      const StateMatrix IdtF = StateMatrix::Identity() + dt * F;
      const Matrix<IDX::SIZE, SRIMU> dtG = dt * G;

      prior_.segment<IDX::NPOS>(IDX::POS) +=
        0.5 * dt * prior_.segment<IDX::NVEL>(IDX::VEL);
      prior_.segment<IDX::NVEL>(IDX::VEL) += dt * (GVEC + R * acc);
      prior_.segment<IDX::NPOS>(IDX::POS) +=
        0.5 * dt * prior_.segment<IDX::NVEL>(IDX::VEL);

      prior_.segment<IDX::NATT>(IDX::ATT) +=
        0.5 * dt * Qleftq * Vector<4>(0, omega.x(), omega.y(), omega.z());
      prior_.segment<IDX::NATT>(IDX::ATT).normalize();

      StateMatrix P = IdtF * P_ * IdtF.transpose();
      P.noalias() += dtG * R_imu_ * dtG.transpose();
      P.noalias() += (dt * dt) * Q_;

      P_ = 0.5 * (P + P.transpose());

      t_prior_ += dt;
      if (dt < dt_max) break;
    }
    t_prior_ = t_target;
  }

  return true;
}

bool EkfImu::updateParameters(const std::shared_ptr<EkfImuParameters>& params) {
  if (!params || !params->valid()) return false;

  std::lock_guard<std::mutex> lock(mutex_);

  params_ = params;

  Q_ = (StateVector() << params_->Q_pos, params_->Q_att, params_->Q_vel,
        params_->Q_bome, params_->Q_bacc)
         .finished()
         .asDiagonal();

  Q_init_ = (StateVector() << params_->Q_init_pos, params_->Q_init_att,
             params_->Q_init_vel, params_->Q_init_bome, params_->Q_init_bacc)
              .finished()
              .asDiagonal();

  R_pose_ = (Vector<SRPOSE>() << params_->R_pos, params_->R_att)
              .finished()
              .asDiagonal();
  R_imu_ = (Vector<SRIMU>() << params_->R_acc, params_->R_omega)
             .finished()
             .asDiagonal();

  return true;
}

bool EkfImu::vectorToState(const Scalar t, const StateVector& x,
                           QuadState* const state) const {
  if (!std::isfinite(t) || !x.allFinite() || state == nullptr) return false;
  state->t = t;
  state->p = x.segment<IDX::NPOS>(IDX::POS);
  state->qx = x.segment<IDX::NATT>(IDX::ATT);
  state->v = x.segment<IDX::NVEL>(IDX::VEL);
  state->w = imu_last_.omega;
  state->a = GVEC + state->R() * imu_last_.acc;
  state->bw = x.segment<IDX::NBOME>(IDX::BOME);
  state->ba = x.segment<IDX::NBACC>(IDX::BACC);
  state->qx.normalize();
  state->mot = motor_speeds_;

  return true;
}

bool EkfImu::stateToVector(const QuadState& state, Scalar* const t,
                           StateVector* const x) const {
  if (!state.valid() || t == nullptr || x == nullptr) return false;

  *t = state.t;
  x->segment<IDX::NPOS>(IDX::POS) = state.p;
  x->segment<IDX::NATT>(IDX::ATT) = state.qx;
  x->segment<IDX::NVEL>(IDX::VEL) = state.v;
  x->segment<IDX::NBOME>(IDX::BOME) = state.bw;
  x->segment<IDX::NBACC>(IDX::BACC) = state.ba;

  return true;
}

}  // namespace agi
