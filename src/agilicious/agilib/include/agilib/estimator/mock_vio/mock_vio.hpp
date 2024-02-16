#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <random>

#include "agilib/estimator/estimator_base.hpp"
#include "agilib/estimator/mock_vio/mock_vio_params.hpp"
#include "agilib/math/integrator_euler.hpp"
#include "agilib/types/pose.hpp"
#include "agilib/types/quad_state.hpp"
#include "agilib/types/quadrotor.hpp"

namespace agi {

class MockVio : public EstimatorBase {
 public:
  MockVio(const Quadrotor& quad, const std::shared_ptr<MockVioParams>& params);

  virtual bool initialize(const QuadState& state) override;

  virtual bool addPose(const Pose& pose) override { return false; }
  virtual bool addState(const QuadState& pose) override;
  virtual bool addImu(const ImuSample& imu) override;
  inline virtual bool addMotorSpeeds(const Vector<4>& speeds) override {
    return false;
  }

  virtual bool getAt(const Scalar t, QuadState* const state) override;

  void reset();
  virtual bool healthy() const override;

 protected:
  class ImuSampleWithBias : public ImuSample {
   public:
    using ImuSample::ImuSample;
    ImuSampleWithBias(const ImuSample& imu) : ImuSample(imu) {}
    ImuSampleWithBias& operator=(const ImuSample& rhs) {
      this->t = rhs.t;
      this->acc = rhs.acc;
      this->omega = rhs.omega;
      return *this;
    }

    Vector<3> acc_bias = Vector<3>::Zero();
    Vector<3> omega_bias = Vector<3>::Zero();
  };

  bool updateVioStateWithLatency(const Scalar t);
  bool updateVioState(const Scalar t);
  bool updateRiseState(const Scalar t);
  bool updateStateWithImu(const Scalar t, const bool repropagate);
  Vector<3> randomVector(const Vector<3>& standard_deviation);

  std::shared_ptr<MockVioParams> params_;
  const Quadrotor quad_;
  IntegratorEuler integrator_;

  Scalar t_last_vio_update_{0.0};
  QuadState last_vio_state_;
  QuadState state_;
  ImuSampleWithBias last_imu_sample_;
  Vector<3> vio_pos_drift_ = Vector<3>::Zero();
  Vector<3> vio_pos_drift_dynamic_vel_ = Vector<3>::Zero();
  Vector<3> vio_pos_drift_static_vel_ = Vector<3>::Zero();

  static constexpr int BUFFERSIZE = 512;
  Scalar t_last_state_received_{0.0};
  std::deque<QuadState> states_;
  std::mutex states_mtx_;
  std::deque<ImuSampleWithBias> imus_;
  std::mutex imus_mtx_;

  std::random_device rd_{};
  std::mt19937 gen_{rd_()};
};

}  // namespace agi
