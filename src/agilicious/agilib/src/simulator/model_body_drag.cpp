#include "agilib/simulator/model_body_drag.hpp"

namespace agi {

ModelBodyDrag::ModelBodyDrag(Quadrotor quad, const BodyDragParameters& params)
  : ModelBase(quad), params_(params) {}

bool ModelBodyDrag::run(const Ref<const Vector<QS::SIZE>> state,
                        Ref<Vector<QS::SIZE>> derivative) const {
  const Matrix<3, 3> R = Quaternion(state(QS::ATTW), state(QS::ATTX),
                                    state(QS::ATTY), state(QS::ATTZ))
                           .toRotationMatrix();

  const Vector<3> velocity_body =
    R.transpose() * state.segment<QS::NVEL>(QS::VEL);
  const Vector<3> coeff{0.5 * params_.cxy_ * params_.rho_ * params_.ax_,
                        0.5 * params_.cxy_ * params_.rho_ * params_.ay_,
                        0.5 * params_.cz_ * params_.rho_ * params_.az_};
  const Vector<3> force =
    -velocity_body.array() * velocity_body.array().abs() * coeff.array();

  // Set up the derivative
  derivative.segment<QS::NVEL>(QS::VEL) += R * force;

  return true;
}


}  // namespace agi
