#include "agilib/simulator/model_lin_cub_drag.hpp"

namespace agi {

ModelLinCubDrag::ModelLinCubDrag(Quadrotor quad,
                                 const LinCubDragParameters& params)
  : ModelBase(quad), params_(params) {}

bool ModelLinCubDrag::run(const Ref<const Vector<QS::SIZE>> state,
                          Ref<Vector<QS::SIZE>> derivative) const {
  const Matrix<3, 3> R = Quaternion(state(QS::ATTW), state(QS::ATTX),
                                    state(QS::ATTY), state(QS::ATTZ))
                           .toRotationMatrix();

  const Ref<const Vector<3>> vel = state.segment<QS::NVEL>(QS::VEL);

  const Vector<3> velocity_body = R.transpose() * vel;

  Vector<3> force = -velocity_body.array() * params_.lin_drag_coeff_.array() -
                    velocity_body.array() * velocity_body.array() *
                      velocity_body.array() * params_.cub_drag_coeff_.array();

  const Vector<3> ind_lift{
    0.0, 0.0,
    params_.induced_lift_coeff_ * velocity_body.segment(0, 2).squaredNorm()};

  // Set up the derivative
  derivative.segment<QS::NVEL>(QS::VEL) += R * (force + ind_lift) / quad_.m_;

  return true;
}


}  // namespace agi