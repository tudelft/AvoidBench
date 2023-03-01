#include "avoidlib/controller/lowlevel_controller_simple.hpp"

namespace avoidlib {

LowLevelControllerSimple::LowLevelControllerSimple(QuadrotorDynamics quad) {
  updateQuadDynamics(quad);
}

bool LowLevelControllerSimple::updateQuadDynamics(
  const QuadrotorDynamics& quad) {
  quad_dynamics_ = quad;
  B_allocation_ = quad.getAllocationMatrix();
  Kinv_ang_vel_tau_ = quad.getKlqr();
  // std::cout<<"B_allocation_: "<<B_allocation_<<std::endl;
  B_allocation_inv_ = B_allocation_.inverse();
  return true;
}

bool LowLevelControllerSimple::setCommand(const Command& cmd) {
  if (!cmd.valid()) return false;
  cmd_ = cmd;
  if (cmd_.isThrustRates()) {
    cmd_.collective_thrust =
      quad_dynamics_.clampCollectiveThrust(cmd_.collective_thrust);
    cmd_.omega = quad_dynamics_.clampBodyrates(cmd_.omega);
  }

  if (cmd_.isSingleRotorThrusts())
    cmd_.thrusts = quad_dynamics_.clampThrust(cmd_.thrusts);

  return true;
}

bool LowLevelControllerSimple::setKlqr(const Ref<Matrix<3, 6>> K_lqr)
{
  Kinv_ang_vel_tau_ = K_lqr;
  return true;
}


Vector<4> LowLevelControllerSimple::run(const Ref<Vector<3>> omega_des,
                                        const Ref<Vector<3>> body_torques) {
  Vector<4> motor_thrusts;
  if (!cmd_.isSingleRotorThrusts()) {
    const Scalar force = quad_dynamics_.getMass() * cmd_.collective_thrust;
    Eigen::VectorXd control_error = Eigen::VectorXd::Zero(6);
    control_error.segment(0, 3) = cmd_.omega - omega_des;
    control_error.segment(3, 3) =
      cmd_.omega.cross(quad_dynamics_.getJ() * cmd_.omega) - body_torques;
    const Vector<3> body_torque_des =
      Kinv_ang_vel_tau_ * control_error +
      omega_des.cross(quad_dynamics_.getJ() * omega_des);
    const Vector<4> thrust_torque(force, body_torque_des.x(),
                                  body_torque_des.y(), body_torque_des.z());
    // std::cout<<"cmd_.omega: "<<cmd_.omega.transpose()<<" omega_des: "<<omega_des.transpose()<<std::endl;
    // std::cout<<"thrust_torque: "<<thrust_torque.transpose()<<std::endl;
    motor_thrusts = B_allocation_inv_ * thrust_torque;
  } else {
    motor_thrusts = cmd_.thrusts;
  }
  // std::cout<<"motor_thrusts "<<motor_thrusts.transpose()<<std::endl;
  motor_thrusts = quad_dynamics_.clampThrust(motor_thrusts);
  return motor_thrusts;
}


}  // namespace avoidlib
