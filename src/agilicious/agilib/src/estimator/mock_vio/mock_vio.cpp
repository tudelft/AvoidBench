#include "agilib/estimator/mock_vio/mock_vio.hpp"

#include "agilib/math/gravity.hpp"

namespace agi {

MockVio::MockVio(const Quadrotor& quad,
                 const std::shared_ptr<MockVioParams>& params)
  : EstimatorBase("Mock VIO"),
    params_(params),
    quad_(quad),
    integrator_(quad_.getDynamicsFunction()) {
  state_.setZero();
  last_vio_state_.setZero();
}

bool MockVio::initialize(const QuadState& state) {
  state_ = state;
  last_vio_state_ = state;
  return state_.valid();
}

bool MockVio::addState(const QuadState& state) {
  if (!state.valid()) return false;

  if (!state_.valid()) return initialize(state);

  std::lock_guard<std::mutex> lock(states_mtx_);

  const Scalar dt = state.t - t_last_state_received_;

  const Scalar dt_sqrt = sqrt(dt);
  if (!states_.empty() && !params_->vio_pos_static_drift.isZero()) {
    vio_pos_drift_static_vel_ +=
      dt_sqrt * randomVector(params_->vio_pos_static_drift);
    vio_pos_drift_static_vel_.array() *=
      (1.0 - 1e-2 * params_->vio_pos_static_drift.array());
    vio_pos_drift_ += dt * vio_pos_drift_static_vel_;
  }

  if (!states_.empty() && !params_->vio_pos_dynamic_drift.isZero()) {
    vio_pos_drift_dynamic_vel_ +=
      dt_sqrt * state.v.norm() * randomVector(params_->vio_pos_dynamic_drift);
    vio_pos_drift_dynamic_vel_.array() *=
      (1.0 - 1e-2 * params_->vio_pos_dynamic_drift.array());
    vio_pos_drift_ += dt * vio_pos_drift_dynamic_vel_;
  }

  if (params_->image_dt > 0.0 && dt < params_->image_dt) return true;
  t_last_state_received_ = state.t;
  states_.push_back(state);
  states_.back().bw = last_imu_sample_.omega_bias;
  states_.back().ba = last_imu_sample_.acc_bias;
  if (!params_->vio_omega_bias_noise.isZero())
    states_.back().bw += randomVector(params_->vio_omega_bias_noise);
  if (!params_->vio_acc_bias_noise.isZero())
    states_.back().ba += randomVector(params_->vio_acc_bias_noise);

  if ((int)states_.size() > BUFFERSIZE) logger_.warn("State queue full!");
  while ((int)states_.size() > BUFFERSIZE) states_.pop_front();

  return true;
}

bool MockVio::addImu(const ImuSample& imu) {
  if (!imu.valid()) return false;
  if (imu.t < last_imu_sample_.t + params_->imu_dt) return true;

  // Prepare measurement and add noise and bias.
  const Scalar t_last_imu = last_imu_sample_.t;
  last_imu_sample_.t = imu.t;
  last_imu_sample_.acc = imu.acc;
  last_imu_sample_.omega = imu.omega;

  if (!params_->imu_omega_noise.isZero())
    last_imu_sample_.omega += randomVector(params_->imu_omega_noise);

  if (!params_->imu_acc_noise.isZero())
    last_imu_sample_.acc += randomVector(params_->imu_acc_noise);

  const Scalar dt_sqrt = sqrt(imu.t - t_last_imu);
  if (!params_->imu_omega_bias.isZero()) {
    if (std::isfinite(t_last_imu))
      last_imu_sample_.omega_bias +=
        dt_sqrt * randomVector(params_->imu_omega_bias);
  }

  if (!params_->imu_acc_bias.isZero()) {
    if (std::isfinite(t_last_imu))
      last_imu_sample_.acc_bias +=
        dt_sqrt * randomVector(params_->imu_acc_bias);
  }

  // Add to queue.
  if (!params_->use_imu) return true;
  imus_.push_back(last_imu_sample_);

  if ((int)imus_.size() > BUFFERSIZE) {
    logger_.warn("IMU queue full!");
    std::lock_guard<std::mutex> lock(imus_mtx_);
    while ((int)imus_.size() > BUFFERSIZE) imus_.pop_front();
  }

  return true;
}

bool MockVio::getAt(const Scalar t, QuadState* const state) {
  if (state == nullptr) return false;

  if (t <= state_.t) {
    *state = state_;
    return true;
  }
  const bool updated_vio_state = updateVioStateWithLatency(t);
  updateStateWithImu(t, updated_vio_state);

  if (!state_.valid()) {
    logger_.warn("State not valid!\n");
    logger_ << state_;
    return false;
  }

  *state = state_;

  return true;
}

bool MockVio::updateVioStateWithLatency(const Scalar t) {
  if (t < t_last_vio_update_ + params_->vio_dt) return false;

  if (params_->rise <= 0.0) {
    updateVioState(t);
  } else {
    updateRiseState(t);
  }

  if (!params_->vio_pos_noise.isZero())
    last_vio_state_.p += randomVector(params_->vio_pos_noise);

  if (!params_->vio_vel_noise.isZero())
    last_vio_state_.v += randomVector(params_->vio_vel_noise);

  if (!params_->vio_att_noise.isZero()) {
    const Vector<3> qe = randomVector(params_->vio_att_noise);
    const Quaternion qrand(sqrt(1.0 - qe.transpose() * qe), qe.x(), qe.y(),
                           qe.z());
    last_vio_state_.q(last_vio_state_.q() * qrand);
  }

  last_vio_state_.p += vio_pos_drift_;

  return true;
}

bool MockVio::updateVioState(const Scalar t) {
  const Scalar t_vio = t - params_->vio_latency;

  std::lock_guard<std::mutex> lock(states_mtx_);

  if (states_.empty()) return false;

  auto next_vio_state = std::lower_bound(states_.begin(), states_.end(), t_vio);

  if (next_vio_state > states_.begin()) --next_vio_state;
  if (next_vio_state >= states_.end()) return false;

  if (next_vio_state->t > t_vio) return false;

  last_vio_state_ = *next_vio_state;
  t_last_vio_update_ = t;
  states_.erase(states_.begin(), next_vio_state);

  return true;
}

bool MockVio::updateRiseState(const Scalar t) {
  const Scalar t_vio = t - params_->vio_latency;

  std::lock_guard<std::mutex> lock(states_mtx_);

  if (states_.empty()) return false;

  auto state_ptr = states_.begin();
  const auto first_state_after_latency =
    std::lower_bound(states_.begin(), states_.end(), t_vio);
  for (; t_last_vio_update_ <= t;) {
    auto next_state_ptr = std::lower_bound(state_ptr, first_state_after_latency,
                                           t_last_vio_update_);
    if (next_state_ptr > state_ptr) --next_state_ptr;

    last_vio_state_.t = next_state_ptr->t;
    last_vio_state_.p +=
      params_->rise * (next_state_ptr->p - last_vio_state_.p);
    last_vio_state_.v +=
      params_->rise * (next_state_ptr->v - last_vio_state_.v);
    last_vio_state_.q(
      last_vio_state_.q().slerp(params_->rise, next_state_ptr->q()));
    last_vio_state_.bw +=
      params_->rise * (next_state_ptr->bw - last_vio_state_.bw);
    last_vio_state_.ba +=
      params_->rise * (next_state_ptr->ba - last_vio_state_.ba);

    t_last_vio_update_ += params_->vio_dt;
    state_ptr = next_state_ptr;
  }

  states_.erase(states_.begin(), state_ptr);

  return true;
}

bool MockVio::updateStateWithImu(const Scalar t, const bool repropagate) {
  // No propagation with IMU.
  if (!params_->use_imu) {
    state_ = last_vio_state_;
    state_.t = t;
    if (last_imu_sample_.valid()) {
      state_.w = last_imu_sample_.omega;
      state_.a = state_.R() * last_imu_sample_.acc + GVEC;
    }
    return true;
  }

  static std::deque<ImuSampleWithBias> imus;
  {
    std::lock_guard<std::mutex> lock(imus_mtx_);
    imus = imus_;
  }

  // Now with propagation. If need to repropagate, reset state_ and start.
  if (repropagate) state_ = last_vio_state_;

  // First relevant sample.
  const Scalar t_start = state_.t;
  auto imu_ptr = std::lower_bound(
    imus.begin(), imus.end(), state_.t,
    [](const ImuSampleWithBias& imu, const Scalar tt) { return imu.t <= tt; });

  // Propagate using all relevant IMU samples.
  while (state_.t < t) {
    QuadState init_state = state_;

    if (imu_ptr < imus.end()) {
      state_.t = std::min(imu_ptr->t, t);
      init_state.w = imu_ptr->omega + imu_ptr->omega_bias - state_.bw;
      init_state.a =
        state_.R() * (imu_ptr->acc + imu_ptr->acc_bias - state_.ba) + GVEC;
    } else {
      state_.t = t;
    }

    integrator_.integrate(init_state, &state_);

    if (imu_ptr < imus.end() && imu_ptr->t < t) ++imu_ptr;
  }

  // Set omega and acc directly, since those are measured quantities.
  if (last_imu_sample_.valid()) {
    state_.w = last_imu_sample_.omega;
    state_.a = state_.R() * (last_imu_sample_.acc) + GVEC;
  }

  // If we repropagated, we can forget about old IMU samples.
  if (repropagate) {
    std::lock_guard<std::mutex> lock(imus_mtx_);
    const auto delete_to_ptr =
      std::lower_bound(imus_.begin(), imus_.end(), t_start);
    imus_.erase(imus_.begin(), delete_to_ptr);
  }

  return true;
}

void MockVio::reset() {
  std::lock_guard<std::mutex> lock_states(states_mtx_);
  std::lock_guard<std::mutex> lock_imus(imus_mtx_);

  last_vio_state_ = QuadState();
  state_ = QuadState();
  t_last_vio_update_ = 0.0;
  t_last_state_received_ = 0.0;

  vio_pos_drift_.setZero();
  vio_pos_drift_dynamic_vel_.setZero();
  vio_pos_drift_static_vel_.setZero();

  last_imu_sample_ = ImuSampleWithBias();

  states_.clear();
  imus_.clear();
}

bool MockVio::healthy() const { return state_.valid(); }

Vector<3> MockVio::randomVector(const Vector<3>& standard_deviation) {
  return Vector<3>(
    std::normal_distribution<Scalar>(0.0, standard_deviation(0))(gen_),
    std::normal_distribution<Scalar>(0.0, standard_deviation(1))(gen_),
    std::normal_distribution<Scalar>(0.0, standard_deviation(2))(gen_));
}

}  // namespace agi
