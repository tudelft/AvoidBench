#pragma once

#include <stdlib.h>

// avoidlib
#include "avoidlib/common/command.hpp"
#include "avoidlib/common/integrator_rk4.hpp"
#include "avoidlib/common/logger.hpp"
#include "avoidlib/common/types.hpp"
#include "avoidlib/common/utils.hpp"
#include "avoidlib/controller/lowlevel_controller_betaflight.hpp"
#include "avoidlib/controller/lowlevel_controller_simple.hpp"
#include "avoidlib/dynamics/quadrotor_dynamics.hpp"
#include "avoidlib/objects/object_base.hpp"
#include "avoidlib/sensors/imu.hpp"
#include "avoidlib/sensors/rgb_camera.hpp"
namespace avoidlib {

class Quadrotor : ObjectBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Quadrotor(const QuadrotorDynamics& dynamics = QuadrotorDynamics(1.0));
  Quadrotor(const std::string& cfg_path);
  ~Quadrotor();

  // reset
  bool reset() override;
  bool reset(const QuadState& state);
  bool reset(const QuadState& state, const Vector<4> motor_omega);
  void init();

  // run the quadrotor
  bool run(const Scalar dt) override;
  bool run(const Command& cmd, const Scalar dt);

  // public get functions
  bool getState(QuadState* const state) const;
  bool getMotorThrusts(Ref<Vector<4>> motor_thrusts) const;
  Vector<4> getMotorThrusts(void) const;
  bool getMotorOmega(Ref<Vector<4>> motor_omega) const;
  Vector<4> getMotorOmega(void) const;
  bool getDynamics(QuadrotorDynamics* const dynamics) const;

  const QuadrotorDynamics& getDynamics();
  Vector<3> getSize(void) const;
  Vector<3> getPosition(void) const;
  Quaternion getQuaternion(void) const;
  std::vector<std::shared_ptr<RGBCamera>> getCameras(void) const;
  bool getCamera(const size_t cam_id, std::shared_ptr<RGBCamera> camera) const;
  int getNumCamera() const;
  bool getCollision() const;

  // public set functions
  bool setState(const QuadState& state);
  bool setCommand(const Command& cmd);
  bool updateDynamics(const QuadrotorDynamics& dynamics);
  bool addRGBCamera(std::shared_ptr<RGBCamera> camera);

  bool updateBodyDragCoeff1(const Vector<3> cd1);

  // simulate motors
  void runMotors(const Scalar sim_dt, const Vector<4>& motor_thrust_des);

  // constrain world box
  bool setWorldBox(const Ref<Matrix<3, 2>> box);
  bool constrainInWorldBox(const QuadState& old_state);

  //
  inline Scalar getMass(void) { return dynamics_.getMass(); };
  inline void setSize(const Ref<Vector<3>> size) { size_ = size; };
  inline void setCollision(const bool collision) { collision_ = collision; };

 private:
  // quadrotor dynamics, integrators
  QuadrotorDynamics dynamics_;
  IMU imu_;
  LowLevelControllerSimple ctrl_;
  // LowLevelControllerBetaflight ctrl_;
  std::unique_ptr<IntegratorRK4> integrator_ptr_;
  std::vector<std::shared_ptr<RGBCamera>> rgb_cameras_;
  Logger logger_{"Quadrotor"};

  // quad control command
  Command cmd_;

  // quad state
  QuadState state_;
  Vector<3> size_;
  bool collision_;

  // auxiliar variablers
  Vector<4> motor_omega_;
  Vector<4> motor_thrusts_;
  Matrix<4, 4> B_allocation_;
  Matrix<4, 4> B_allocation_inv_;

  // P gain for body-rate control
  const Matrix<3, 3> Kinv_ang_vel_tau_ =
    Vector<3>(16.6, 16.6, 5.0).asDiagonal();

  // gravity
  const Vector<3> gz_{0.0, 0.0, Gz};

  // auxiliary variables
  Matrix<3, 2> world_box_;
};


}  // namespace avoidlib
