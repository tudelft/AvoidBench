#include "agilib/math/integrator_symplectic_euler.hpp"

namespace agi {

bool IntegratorSymplecticEuler::step(const Ref<const Vector<>> initial_state,
                                     const Scalar dt,
                                     Ref<Vector<>> final_state) const {
  Vector<> derivative(initial_state.rows());
  final_state = initial_state;

  if (!this->dynamics_(final_state, derivative)) return false;

  final_state.segment<QS::NVEL>(QS::VEL) =
    initial_state.segment<QS::NVEL>(QS::VEL) +
    dt * derivative.segment<QS::NVEL>(QS::VEL);
  final_state.segment<QS::NOME>(QS::OME) =
    initial_state.segment<QS::NOME>(QS::OME) +
    dt * derivative.segment<QS::NOME>(QS::OME);
  final_state.segment<QS::NMOT>(QS::MOT) =
    initial_state.segment<QS::NMOT>(QS::MOT) +
    dt * derivative.segment<QS::NMOT>(QS::MOT);

  if (!this->dynamics_(final_state, derivative)) {
    return false;
  }
  final_state.segment<QS::NPOS>(QS::POS) =
    initial_state.segment<QS::NPOS>(QS::POS) +
    dt * derivative.segment<QS::NPOS>(QS::POS);
  final_state.segment<QS::NATT>(QS::ATT) =
    initial_state.segment<QS::NATT>(QS::ATT) +
    dt * derivative.segment<QS::NATT>(QS::ATT);

  return true;
}

}  // namespace agi
