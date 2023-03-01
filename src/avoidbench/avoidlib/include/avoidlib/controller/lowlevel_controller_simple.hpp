#pragma once

#include "avoidlib/common/command.hpp"
#include "avoidlib/common/types.hpp"
#include "avoidlib/dynamics/quadrotor_dynamics.hpp"

namespace avoidlib {

class LowLevelControllerSimple {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LowLevelControllerSimple(QuadrotorDynamics quad_dynamics);
  bool setCommand(const Command& cmd);
  Vector<4> run(const Ref<Vector<3>> omega,
                const Ref<Vector<3>> body_torques);
  bool updateQuadDynamics(const QuadrotorDynamics& quad_dynamics);
  bool setKlqr(const Ref<Matrix<3, 6>> K_lqr);

 private:
  // Quadrotor properties
  Matrix<4, 4> B_allocation_;
  Matrix<4, 4> B_allocation_inv_;

  // P gain for body rate control
  Matrix<3, 6> Kinv_ang_vel_tau_;

  // Quadcopter to which the controller is applied
  QuadrotorDynamics quad_dynamics_;

  // Motor speeds calculated by the controller
  Vector<4> motor_omega_des_;

  // Command
  Command cmd_;
};

}  // namespace avoidlib
