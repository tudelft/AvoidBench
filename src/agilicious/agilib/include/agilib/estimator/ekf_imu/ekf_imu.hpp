#pragma once

#include <deque>
#include <mutex>

#include "agilib/estimator/ekf_imu/ekf_imu_params.hpp"
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
class EkfImu : public EstimatorBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EkfImu(const std::shared_ptr<EkfImuParameters>& params =
           std::shared_ptr<EkfImuParameters>());
  ~EkfImu();

  bool getAt(const Scalar t, QuadState* const state) override;

  bool initialize(const QuadState& state) override;

  bool addPose(const Pose& pose) override;
  bool addState(const QuadState& pose) override;
  bool addImu(const ImuSample& imu) override;
  bool addMotorSpeeds(const Vector<4>& speeds) override;

  bool updateParameters(const std::shared_ptr<EkfImuParameters>& params_in);

  bool healthy() const override;

  inline void printTiming() { printTimings(false); }
  void printTimings(const bool all = true) const;
  void logTiming() const override;

 private:
  enum IDX : int {
    POS = 0,
    POSX = 0,
    POSY = 1,
    POSZ = 2,
    NPOS = 3,
    ATT = 3,
    ATTW = 3,
    ATTX = 4,
    ATTY = 5,
    ATTZ = 6,
    NATT = 4,
    VEL = 7,
    VELX = 7,
    VELY = 8,
    VELZ = 9,
    NVEL = 3,
    BOME = 10,
    BOMEX = 10,
    BOMEY = 11,
    BOMEZ = 12,
    NBOME = 3,
    BACC = 13,
    BACCX = 13,
    BACCY = 14,
    BACCZ = 15,
    NBACC = 3,
    SIZE = 16,
  };

  using StateVector = Vector<IDX::SIZE>;
  using StateMatrix = Matrix<IDX::SIZE, IDX::SIZE>;
  static constexpr int MAX_QUEUE_SIZE = 256;
  static constexpr int SRPOSE = 6;  // Size of pose measurement.
  static constexpr int SRIMU = 6;   // Size of IMU measurement.


  bool init(const QuadState& state);
  bool process();
  bool updatePose(const Pose& pose);
  bool propagatePrior(const Scalar time);
  bool propagatePriorAndCovariance(const Scalar time);
  bool vectorToState(const Scalar t, const StateVector& x,
                     QuadState* const state) const;
  bool stateToVector(const QuadState& state, Scalar* const t,
                     StateVector* const x) const;


  // Parameters
  std::shared_ptr<EkfImuParameters> params_;
  StateMatrix Q_ = StateMatrix::Zero();
  StateMatrix Q_init_ = StateMatrix::Zero();
  Matrix<SRPOSE, SRPOSE> R_pose_;
  Matrix<SRIMU, SRIMU> R_imu_;
  ImuSample imu_last_;

  // Working variables
  Scalar t_prior_{NAN}, t_posterior_{NAN};
  StateVector prior_ = StateVector::Zero();
  StateVector posterior_ = StateVector::Zero();
  StateMatrix P_ = StateMatrix::Zero();
  Vector<4> motor_speeds_{0, 0, 0, 0};

  // Thread Safety
  std::mutex mutex_;

  std::deque<Pose> poses_;
  std::deque<ImuSample> imus_;

  // Logger and timers
  Timer timer_process_{"Process"};
  Timer timer_update_pose_{"Update Pose"};
  Timer timer_compute_gain_{"Kalman gain computation"};
  Timer timer_propagation_{"Propagation Prior"};
  Timer timer_jacobian_{"Propagation Jacobian"};
};

}  // namespace agi
