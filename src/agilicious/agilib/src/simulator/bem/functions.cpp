#include "agilib/simulator/bem/functions.hpp"

namespace agi {

Array<QS::NMOT, 15> IntegrandPsi::evaluate(const Ref<const ArrayVector<15>> psi,
                                           const void* const param) const {
  const Scalar r = ((IntegrandParam*)param)->radius;
  const ArrayVector<QS::NMOT> vind = ((IntegrandParam*)param)->vind;
  const Matrix<1, 15> s_psi = psi.sin();
  const Matrix<1, 15> c_psi = psi.cos();
  const Array<QS::NMOT, 15> beta =
    (-(pstate_->a1s_ * c_psi + pstate_->b1s_ * s_psi)).colwise() + pstate_->a0_;

  const Array<QS::NMOT, 15> u_t =
    (r + pstate_->param_.r_prop_ * (pstate_->mu_ * s_psi).array()).colwise() *
    pstate_->omega_mot_;

  const Array<QS::NMOT, 15> u_p =
    (-((pstate_->vver_ * c_psi).array() * beta +
       (pstate_->a1s_ * s_psi + pstate_->b1s_ * c_psi).array().colwise() *
         pstate_->omega_mot_ * r))
      .array()
      .colwise() +
    (pstate_->vver_.array() - vind);

  const Array<QS::NMOT, 15> phi =
    u_p.array().binaryExpr(u_t.array(), ApproxAtan2<Scalar>());

  const Array<QS::NMOT, 15> alpha =
    pstate_->param_.theta0_ +
    pstate_->param_.theta1_ * r / pstate_->param_.r_prop_ + phi;
  const Array<QS::NMOT, 15> s_alpha = alpha.sin();
  const Array<QS::NMOT, 15> c_alpha = alpha.cos();
  const Array<QS::NMOT, 15> cl =
    pstate_->param_.cl_ * (s_alpha * c_alpha + 0.07);
  const Array<QS::NMOT, 15> cd = pstate_->param_.cd_ * s_alpha.square();

  const Scalar c =
    pstate_->param_.ci_ +
    r / pstate_->param_.r_prop_ * (pstate_->param_.co_ - pstate_->param_.ci_);
  const Array<QS::NMOT, 15> u_sqr = u_t.square() + u_p.square();
  const Array<QS::NMOT, 15> d_lift = u_sqr * cl * c;
  const Array<QS::NMOT, 15> d_drag = u_sqr * cd * c;

  switch (type_) {
    case THRUST:
      return d_lift * phi.cos() + d_drag * phi.sin();
    case TORQUE:
      return r * (-d_lift * phi.sin() + d_drag * phi.cos());
    case HFORCE:
      return (-d_lift * phi.sin() + d_drag * phi.cos()).rowwise() *
             s_psi.array();
    default:
      return Array<QS::NMOT, 15>::Zero();
  }
}


Array<4, 15> IntegrandR::evaluate(const Ref<const ArrayVector<15>> r,
                                  const void* const param) const {
  IntegrandParam int_param;
  if (param == nullptr) {
    int_param.vind = pstate_->vind_;
  } else {
    int_param.vind = ((IntegrandParam*)param)->vind;
  }

  Array<4, 15> retVal = Array<4, 15>::Zero();

#pragma omp parallel for default(none) shared(retVal, r, gk_) \
  firstprivate(int_param) num_threads(5) proc_bind(spread) schedule(static)
  for (int i = 0; i < 15; ++i) {
    int_param.radius = r(i);
    retVal.col(i) =
      gk_.integrate(*fPsi_, 0, 2 * M_PI, (const void* const) & int_param)
        .result_;
  }
  return pstate_->param_.b_ * pstate_->param_.rho_ / (4 * M_PI) * retVal;
}


ArrayVector<4> ThrustFunction::evaluate(
  const Ref<const ArrayVector<4>> vind) const {
  IntegrandParam int_param{vind, 0};
  const ArrayVector<4> t_bem = gk_
                                 .integrate(*fR_, 0, pstate_->param_.r_prop_,
                                            (const void* const) & int_param)
                                 .result_;
  const ArrayVector<4> tmp = pstate_->vver_.array() - vind;
  const ArrayVector<4> t_mom =
    2 * pstate_->param_.rho_ * pstate_->param_.prop_area_ * vind *
    (pstate_->vhor_.array() * pstate_->vhor_.array() + tmp * tmp).sqrt();
  return t_bem - t_mom;
}

}  // namespace agi
