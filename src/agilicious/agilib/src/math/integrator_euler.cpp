#include "agilib/math/integrator_euler.hpp"

namespace agi {

bool IntegratorEuler::step(const Ref<const Vector<>> initial_state,
                           const Scalar dt, Ref<Vector<>> final_state) const {
  Vector<> derivative(initial_state.rows());
  if (!this->dynamics_(initial_state, derivative)) return false;

  final_state = initial_state + dt * derivative;

  return true;
}

}  // namespace agi