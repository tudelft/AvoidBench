#include "agilib/simulator/model_propeller_bem.hpp"

namespace agi {

ModelPropellerBEM::ModelPropellerBEM(Quadrotor quad,
                                     const BEMParameters& params)
  : ModelBase(quad),
    prop_state_(std::make_shared<PropellerState>(params)),
    f_psi_thrust_(std::make_shared<IntegrandPsi>(IntegrandPsi::THRUST)),
    f_psi_torque_(std::make_shared<IntegrandPsi>(IntegrandPsi::TORQUE)),
    f_psi_hforce_(std::make_shared<IntegrandPsi>(IntegrandPsi::HFORCE)),
    f_r_thrust_(std::make_shared<IntegrandR>()),
    f_r_torque_(std::make_shared<IntegrandR>()),
    f_r_hforce_(std::make_shared<IntegrandR>()),
    f_v1_(std::make_shared<ThrustFunction>()),
    vind_solver_() {
  updateQuad(quad);
  vind_ = Vector<QS::NMOT>::Zero();
  vind_h_ = Vector<QS::NMOT>::Zero();
  // Order: 1 - front right (beta 2)
  //        2 - back left (beta 3)
  //        3 - back right (beta 1)
  //        4 - front left (beta 4)
}


ModelPropellerBEM::~ModelPropellerBEM() { logger_.debug() << timer_; }


bool ModelPropellerBEM::run(const Ref<const Vector<QS::SIZE>> state,
                            Ref<Vector<QS::SIZE>> derivative) const {
  if (!state.segment<QS::DYN>(0).allFinite()) return false;

  ScopedTicToc tictoc(timer_);

  // Everything is calculated in FRD
  prop_state_->update(state, quad_.t_BM_);
  f_psi_thrust_->setState(prop_state_);
  f_psi_torque_->setState(prop_state_);
  f_psi_hforce_->setState(prop_state_);

  f_r_thrust_->setState(prop_state_, f_psi_thrust_);
  f_r_torque_->setState(prop_state_, f_psi_torque_);
  f_r_hforce_->setState(prop_state_, f_psi_hforce_);
  f_v1_->setState(prop_state_, f_r_thrust_);
  vind_solver_.find_root(*f_v1_, -20, 30, vind_, true, vind_);
  prop_state_->vind_ = vind_;

  ArrayVector<4> tmp = prop_state_->vver_.array() / prop_state_->vind_.array();
  if ((tmp >= 0.01).any() && (tmp <= 2).any()) {
    const Scalar k0 = 1;
    const Scalar k1 = -1.125;
    const Scalar k2 = -1.372;
    const Scalar k3 = -1.718;
    const Scalar k4 = -0.655;
    const ArrayVector<4> vz = -prop_state_->vver_;
    const ArrayVector<4> vtot_old = prop_state_->vtot_;

    prop_state_->vtot_ = prop_state_->vhor_;
    prop_state_->vver_.setZero();
    f_v1_->setState(prop_state_, f_r_thrust_);
    vind_solver_.find_root(*f_v1_, -20, 30, vind_h_, true, vind_h_);
    const ArrayVector<4> vzvh = vz / vind_h_;
    const ArrayVector<4> vind2 =
      vind_h_ *
      (k0 + k1 * vzvh + k2 * vzvh * vzvh + k3 * vzvh.pow(3) + k4 * vzvh.pow(4));

    prop_state_->vind_ = prop_state_->vind_.array().max(vind2);
    prop_state_->vver_ = -vz;
    prop_state_->vtot_ = vtot_old;
    for (int i = 0; i < QS::NMOT; ++i)
      if (tmp(i) >= 0.01 && tmp(i) <= 2) {
        prop_state_->vind_(i) =
          std::fmin(prop_state_->vind_(i), 2 * vind_h_(i));
      }
  }

  prop_state_->thrust_ =
    gk_.integrate(*f_r_thrust_, 0, prop_state_->param_.r_prop_).result_;
  prop_state_->torque_ =
    gk_.integrate(*f_r_torque_, 0, prop_state_->param_.r_prop_).result_;
  // For a single propeller, the BEM is accurate (verified against published
  // wind-tunnel data, Gill et al. "Propeller Thrust and Drag in Forward
  // Flight"). For the drone, the BEM underestimates the drag which is corrected
  // by the factor 3.0. The factor was identified using real-world flight data.
  prop_state_->hforce_ =
    gk_.integrate(*f_r_hforce_, 0, prop_state_->param_.r_prop_).result_ * 3.0;
  prop_state_->calculateFlapping();

  // Combine all four propellers in FLU frame!
  Scalar power = 0;
  Vector<3> force;
  Vector<3> torque;
  force.setZero();
  torque.setZero();
  for (int i = 0; i < QS::NMOT; ++i) {
    Matrix<3, 3> mot_rot =
      Eigen::AngleAxisd{
        std::atan2(prop_state_->velocity_(1, i), prop_state_->velocity_(0, i)),
        Eigen::Vector3d::UnitZ()}
        .toRotationMatrix();
    const int cw = (i >= 2 ? 1 : -1);
    const Scalar pthrust = prop_state_->thrust_(i);
    const Vector<3> f =
      mot_rot * Vector<3>{-(prop_state_->hforce_(i) +
                            std::sin(prop_state_->a1s_(i)) * pthrust),
                          cw * std::sin(prop_state_->b1s_(i)) * pthrust,
                          -std::cos(prop_state_->a0_(i)) * pthrust};
    mot_rot* Vector<3>{0, 0, -std::cos(prop_state_->a0_(i)) * pthrust};
    force += (f.array() * prop_state_->flu_conv_frd_).matrix();

    const Scalar ptorque = prop_state_->torque_(i);
    power += std::abs(ptorque * state.segment<QS::NMOT>(QS::MOT)(i));
    const Vector<3> t =
      mot_rot *
      Vector<3>{cw * prop_state_->param_.k_spring_ * prop_state_->b1s_(i),
                prop_state_->param_.k_spring_ * prop_state_->a1s_(i),
                -cw * ptorque};
    torque += (t.array() * prop_state_->flu_conv_frd_).matrix() +
              quad_.t_BM_.col(i).cross(
                (f.array() * prop_state_->flu_conv_frd_).matrix());
  }
  force.z() = force.z() * thrust_scale_;

  // Convert force and torque to acceleration and omega_dot
  derivative.segment<QS::NVEL>(QS::VEL) +=
    prop_state_->rot_ * force / quad_.m_ + GVEC;
  derivative.segment<QS::NOME>(QS::OME) += quad_.J_inv_ * torque;

  return true;
}

}  // namespace agi
