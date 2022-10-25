#include "avoidlib/common/integrator_euler.hpp"

namespace avoidlib {

bool IntegratorEuler::step(const Ref<const Vector<>> initial, const Scalar dt,
                           Ref<Vector<>> final) const {
  Vector<> derivative(initial.rows());
  if (!this->dynamics_(initial, derivative)) return false;

  final = initial + dt * derivative;

  return true;
}

}  // namespace avoidlib