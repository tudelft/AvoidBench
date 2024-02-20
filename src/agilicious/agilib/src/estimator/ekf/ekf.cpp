#include "agilib/estimator/ekf/ekf.hpp"

#include <iostream>

#include "agilib/math/gravity.hpp"

namespace agi {

Ekf::Ekf(const Quadrotor& quad, const std::shared_ptr<EkfParameters>& params)
  : EstimatorBase("EKF"),
    quad_(quad),
    integrator_(quad_.getDynamicsFunction()) {
  if (!updateParameters(params)) logger_.error("Could not load parameters!");
}


Ekf::~Ekf() { printTimings(); }

bool Ekf::getAt(const Scalar t, QuadState* const state) {
  if (state == nullptr) return false;
  if (std::isnan(t)) return false;
  if (!posterior_.valid()) return false;

  std::lock_guard<std::mutex> lock(mutex_);
  if (params_->update_on_get) process();

  // Catch trivial cases...
  if (t <= prior_.t) {  // ...request at prior
    *state = prior_;
    state->mot = motor_speeds_;
    return true;
  }

  const bool ret = propagatePrior(t);
  *state = prior_;
  state->mot = motor_speeds_;
  return ret;
}

bool Ekf::initialize(const QuadState& state) {
  std::lock_guard<std::mutex> lock(mutex_);
  return init(state);
}

bool Ekf::init(const QuadState& state) {
  if (!state.valid()) return false;

  if (!poses_.empty()) {
    const std::deque<Pose>::iterator first_valid_pose = std::lower_bound(
      poses_.begin(), poses_.end(), state.t,
      [](const Pose& pose, const Scalar t) { return pose.t < t; });

    poses_.erase(poses_.begin(), first_valid_pose);
  }

  if (!imus_.empty()) {
    const std::deque<ImuSample>::iterator first_valid_imu = std::lower_bound(
      imus_.begin(), imus_.end(), state.t,
      [](const ImuSample& imu, const Scalar t) { return imu.t < t; });

    imus_.erase(imus_.begin(), first_valid_imu);
  }

  prior_ = state;
  posterior_ = state;
  P_ = Q_init_;

  return true;
}

bool Ekf::initSteady(const QuadState& state) {
  const QuadState steady_state(state.t, state.p, state.getYaw());
  return init(steady_state);
}

bool Ekf::addPose(const Pose& pose) {
  if (!pose.valid()) return false;

  std::lock_guard<std::mutex> lock(mutex_);

  if (!posterior_.valid()) {
    posterior_.setZero();
    posterior_.t = pose.t;
    posterior_.p = pose.position;
    posterior_.q(pose.attitude);
    return init(posterior_);
  }

  if (pose.t <= posterior_.t) return false;

  poses_.push_back(pose);
  t_last_pose_ = pose.t;

  while ((int)poses_.size() > MAX_QUEUE_SIZE) poses_.pop_front();

  // logger_.info("Added Pose at %1.6fs!", pose.t);

  process();
  return true;
}

bool Ekf::addState(const QuadState& state) {
  return addPose({state.t, state.p, state.q()});
}

bool Ekf::addImu(const ImuSample& imu) {
  if (!params_->use_imu) return false;
  if (!imu.valid()) return false;

  std::lock_guard<std::mutex> lock(mutex_);

  if (imu.t < posterior_.t) return false;

  imus_.push_back(imu);
  t_last_imu_ = imu.t;
  while ((int)imus_.size() > MAX_QUEUE_SIZE) imus_.pop_front();

  // logger_.info("Added IMU at %1.6fs!", imu.t);

  process();
  return true;
}

bool Ekf::addMotorSpeeds(const Vector<4>& speeds) {
  motor_speeds_ = speeds;
  return true;
}

bool Ekf::healthy() const { return posterior_.valid() && prior_.valid(); }

void Ekf::logTiming() const {
  logger_.addPublishingVariable(
    "timer_process_ms",
    1000.0 * Vector<3>(timer_process_.mean(), timer_process_.min(),
                       timer_process_.max()));
}

void Ekf::printTimings(const bool all) const {
  logger_ << timer_process_;
  if (all) {
    logger_ << timer_update_pose_;
    logger_ << timer_update_imu_;
    logger_ << timer_compute_gain_;
    logger_ << timer_propagation_;
    logger_ << timer_jacobian_;
  }
}

bool Ekf::process() {
  if (params_->enable_timing) timer_process_.tic();
  std::deque<ImuSample>::iterator imu = imus_.begin();
  std::deque<Pose>::iterator pose = poses_.begin();
  if (!params_->use_imu) t_last_imu_ = NAN;

  // logger_.info("Queue Size:  IMU: %03zu   Pose: %03zu", imus_.size(),
  //              poses_.size());

  while (pose < poses_.end()) {  // Iterator over all poses in queue
    while (imu < imus_.end() &&
           imu->t <= pose->t) {  // iterate over earlier IMUs in queue
      if (imu->t >= posterior_.t) {
        if (!updateImu(*imu))
          logger_.error("Could not perform IMU update at %1.3f",
                        imus_.front().t);
      }
      imu++;
    }

    if (std::isnan(t_last_imu_) || pose->t < t_last_imu_) {
      if (!updatePose(*pose))
        logger_.error("Could not perform pose update at %1.3f",
                      imus_.front().t);

      // Pedantic: exit if queues have been emptied in possible filter re-init.
      if (poses_.empty() || (params_->use_imu && imus_.empty())) return true;
      pose++;
    } else if (pose->t > t_last_imu_ + params_->max_update_wait_time) {
      imus_.clear();
      t_last_imu_ = NAN;
    } else {
      break;
    }
  }

  // Remove what was processed, pedantic checking for end-iterator change.
  if (imu > imus_.end()) imu = imus_.end();
  if (pose > poses_.end()) pose = poses_.end();
  imus_.erase(imus_.begin(), imu);
  poses_.erase(poses_.begin(), pose);

  if (params_->enable_timing) timer_process_.toc();
  return true;
}

bool Ekf::updatePose(const Pose& pose) {
  if (params_->enable_timing) timer_update_pose_.tic();
  if (!propagatePriorAndJacobian(pose.t)) {
    logger_.error("Could not propagate to %1.3f", pose.t);
    return false;
  }

  if (params_->jump_pos_threshold > 0.0 || params_->jump_att_threshold > 0.0) {
    const Vector<3> d_pos = pose.position - prior_.p;
    const Quaternion d_q = pose.attitude * prior_.q().conjugate();
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
        logger_ << "Prior Pos: " << prior_.p.transpose() << std::endl;
        logger_ << "Meas Pos:  " << pose.position.transpose() << std::endl;
      }
      if (att_jump) {
        const Quaternion q(prior_.qx(0), prior_.qx(1), prior_.qx(2),
                           prior_.qx(3));
        logger_ << "Prior Att: " << q.coeffs().transpose() << std::endl;
        logger_ << "Meas Att:  " << pose.attitude.coeffs().transpose()
                << std::endl;
      }
      posterior_.setZero();
      posterior_.t = pose.t;
      posterior_.p = pose.position;
      posterior_.q(pose.attitude);
      return init(posterior_);
    }
  }

  // logger_.info("Processing pose at %1.6f", pose.t);

  // Pose update residual and jacobian.
  const Vector<SRPOSE> y = (Vector<SRPOSE>() << prior_.p - pose.position,
                            (pose.attitude.conjugate() * prior_.q()).vec())
                             .finished();

  Matrix<SRPOSE, QS::SIZE> H = Matrix<SRPOSE, QS::SIZE>::Zero();
  H.block<3, 3>(0, QS::IDX::POS) = Matrix<3, 3>::Identity();
  H.block<3, 4>(3, QS::IDX::ATT) =
    Q_left(pose.attitude.conjugate()).bottomRows<3>();

  const bool check = update(y, H, R_pose_);
  if (params_->enable_timing) timer_update_pose_.toc();
  return check;
}


bool Ekf::updateImu(const ImuSample& imu) {
  if (params_->enable_timing) timer_update_imu_.tic();
  if (!propagatePriorAndJacobian(imu.t)) {
    logger_.error("Could not propagate to %1.3f", imu.t);
    return false;
  }

  // logger_.info("Processing IMU at %1.6f", imu.t);

  // Pose update residual and jacobian.
  const Matrix<3, 3> R_BI = prior_.R().transpose();
  const Vector<SRIMU> y = (Vector<SRIMU>() << prior_.w + prior_.bw - imu.omega,
                           R_BI * (prior_.a - GVEC) + prior_.ba - imu.acc)
                            .finished();

  Matrix<SRIMU, QS::SIZE> H = Matrix<SRIMU, QS::SIZE>::Zero();
  H.block<3, 3>(0, QS::IDX::OME) = Matrix<3, 3>::Identity();
  H.block<3, 3>(0, QS::IDX::BOME) = Matrix<3, 3>::Identity();
  H.block<3, 3>(3, QS::IDX::ACC) = R_BI;
  H.block<3, 3>(3, QS::IDX::BACC) = Matrix<3, 3>::Identity();

  const bool check = update(y, H, R_imu_);
  if (params_->enable_timing) timer_update_imu_.toc();
  return check;
}

bool Ekf::propagatePrior(const Scalar t) {
  if (std::isnan(t)) return false;

  if (params_->enable_timing) timer_propagation_.tic();

  if (t < prior_.t) {
    if (t < posterior_.t) return false;
    prior_ = posterior_;
  }

  const QuadState initial = prior_;
  prior_.t = t;
  integrator_.integrate(initial, &prior_);
  sanitize(prior_);

  if (params_->enable_timing) timer_propagation_.toc();
  return true;
}

bool Ekf::propagatePriorAndJacobian(const Scalar t) {
  if (std::isnan(t) || t < posterior_.t) return false;

  if (params_->enable_timing) timer_jacobian_.tic();

  prior_ = posterior_;

  sanitize(prior_);

  const Scalar dt_max = integrator_.dtMax();
  Scalar dt_remaining = t - prior_.t;
  Vector<QS::SIZE> propagated_state;
  SparseMatrix F(QS::SIZE, QS::SIZE);

  while (dt_remaining > 0.0) {
    const Scalar dt = std::min(dt_remaining, dt_max);

    quad_.jacobian(prior_.x, F);
    const Matrix<QS::SIZE, QS::SIZE> P = P_;
    F *= dt;
    P_ += F * P;
    P_ += P * F.transpose();
    P_ += dt * Q_;

    integrator_.step(prior_.x, dt, propagated_state);

    prior_.x = propagated_state;
    prior_.t += dt;
    sanitize(prior_);
    dt_remaining -= dt;
  }

  prior_.t = t;
  sanitize(prior_);
  ;

  if (params_->enable_timing) timer_jacobian_.toc();
  return true;
}

void Ekf::sanitize(QuadState& state) {
  state.tau.setZero();
  state.j.setZero();
  state.s.setZero();
  state.mot.setZero();
  state.motdes.setZero();
  if (!params_->use_imu) {
    state.bw.setZero();
    state.ba.setZero();
  }
  state.linearize();
}

bool Ekf::updateParameters(const std::shared_ptr<EkfParameters>& params) {
  if (!params || !params->valid()) return false;

  std::lock_guard<std::mutex> lock(mutex_);

  params_ = params;

  Q_.setZero();
  R_pose_.setZero();
  R_imu_.setZero();

  const Vector<> q =
    (Vector<QS::MOT>() << params_->Q_pos, params_->Q_att, params_->Q_vel,
     params_->Q_ome, params_->Q_acc, Vector<QuadState::NTAU>::Zero(),
     Vector<QuadState::NJERK>::Zero(), Vector<QuadState::NSNAP>::Zero(),
     params_->Q_bome, params_->Q_bacc)
      .finished();

  Matrix<QS::SIZE, QS::SIZE> Q = Matrix<QS::SIZE, QS::SIZE>::Zero();
  Q.topLeftCorner(q.rows(), q.rows()) = q.asDiagonal();
  Q_ = Q.sparseView();

  const Vector<> q_init =
    (Vector<QS::MOT>() << params_->Q_init_pos, params_->Q_init_att,
     params_->Q_init_vel, params_->Q_init_ome, params_->Q_init_acc,
     Vector<QuadState::NTAU>::Zero(), Vector<QuadState::NJERK>::Zero(),
     Vector<QuadState::NSNAP>::Zero(), params_->Q_init_bome,
     params_->Q_init_bacc)
      .finished();

  Matrix<QS::SIZE, QS::SIZE> Q_init = Matrix<QS::SIZE, QS::SIZE>::Zero();
  Q_init.topLeftCorner(q_init.rows(), q_init.rows()) = q_init.asDiagonal();
  Q_init_ = Q_init.sparseView();

  R_pose_ = (Vector<SRPOSE>() << params_->R_pos, params_->R_att)
              .finished()
              .asDiagonal();
  R_imu_ = (Vector<SRIMU>() << params_->R_acc, params_->R_omega)
             .finished()
             .asDiagonal();

  return true;
}

}  // namespace agi
