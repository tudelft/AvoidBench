#pragma once

#include <deque>
#include <mutex>

#include "agilib/estimator/ekf/ekf_params.hpp"
#include "agilib/estimator/estimator_base.hpp"
#include "agilib/math/integrator_rk4.hpp"
#include "agilib/math/math.hpp"
#include "agilib/math/types.hpp"
#include "agilib/types/imu_sample.hpp"
#include "agilib/types/pose.hpp"
#include "agilib/types/quad_state.hpp"
#include "agilib/types/quadrotor.hpp"
#include "agilib/utils/logger.hpp"
#include "agilib/utils/timer.hpp"

namespace agi {

/// EKF filter for pose measurements.
class Ekf : public EstimatorBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Ekf(const Quadrotor& quad, const std::shared_ptr<EkfParameters>& params =
                               std::shared_ptr<EkfParameters>());
  ~Ekf();

  bool getAt(const Scalar t, QuadState* const state) override;

  bool initialize(const QuadState& state) override;

  bool addPose(const Pose& pose) override;
  bool addState(const QuadState& pose) override;
  bool addImu(const ImuSample& imu) override;
  bool addMotorSpeeds(const Vector<4>& speeds) override;

  bool updateParameters(const std::shared_ptr<EkfParameters>& params);

  bool healthy() const override;

  inline void printTiming() { printTimings(false); }
  void printTimings(const bool all = true) const;
  void logTiming() const override;

 private:
  bool init(const QuadState& state);
  bool initSteady(const QuadState& state);
  bool process();
  template<int N>
  bool update(const Vector<N>& y, const Matrix<N, QuadState::SIZE>& H,
              const Matrix<N, N>& R);
  bool updatePose(const Pose& pose);
  bool updateImu(const ImuSample& imu);
  bool propagatePrior(const Scalar time);
  bool propagatePriorAndJacobian(const Scalar time);
  void sanitize(QuadState& state);

  static constexpr int MAX_QUEUE_SIZE = 256;
  static constexpr int SRPOSE = 6;  // Size of pose measurement.
  static constexpr int SRIMU = 6;   // Size of IMU measurement.

  // Parameters
  std::shared_ptr<EkfParameters> params_;
  SparseMatrix Q_{QuadState::SIZE, QuadState::SIZE};
  SparseMatrix Q_init_{QuadState::SIZE, QuadState::SIZE};
  Matrix<SRPOSE, SRPOSE> R_pose_;
  Matrix<SRIMU, SRIMU> R_imu_;
  Quadrotor quad_;
  IntegratorRK4 integrator_;

  // Working variables
  QuadState posterior_;
  QuadState prior_;
  Vector<4> motor_speeds_{0, 0, 0, 0};
  Matrix<QuadState::SIZE, QuadState::SIZE> P_;

  // Thread Safety
  std::mutex mutex_;

  std::deque<Pose> poses_;
  Scalar t_last_pose_{NAN};
  std::deque<ImuSample> imus_;
  Scalar t_last_imu_{NAN};

  // Logger and timers
  Timer timer_process_{"Process"};
  Timer timer_update_pose_{"Update Pose"};
  Timer timer_update_imu_{"Update IMU"};
  Timer timer_compute_gain_{"Kalman gain computation"};
  Timer timer_propagation_{"Propagation Prior"};
  Timer timer_jacobian_{"Propagation Jacobian"};
};


template<int N>
bool Ekf::update(const Vector<N>& y, const Matrix<N, QuadState::SIZE>& H,
                 const Matrix<N, N>& R) {
  if (params_->enable_timing) timer_compute_gain_.tic();

  const Matrix<QS::SIZE, QS::SIZE> P = 0.5 * (P_ + P_.transpose());
  const Matrix<N, QS::SIZE> HP = H * P;
  const Matrix<N, N> S = HP * H.transpose() + R;
  const Matrix<N, N> S_inv = S.inverse();

  if (!S_inv.allFinite()) return false;

  QuadState new_posterior = prior_;

  const Matrix<QS::SIZE, N> K = P * H.transpose() * S_inv;
  new_posterior.x -= K * y;
  new_posterior.linearize();
  P_ = P - K * HP;

  if (!new_posterior.valid()) return false;

  posterior_ = new_posterior;
  prior_ = new_posterior;

  if (params_->enable_timing) timer_compute_gain_.toc();
  return true;
}

}  // namespace agi
