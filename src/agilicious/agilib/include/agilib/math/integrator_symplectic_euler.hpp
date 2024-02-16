#pragma once

#include "agilib/math/integrator_base.hpp"

namespace agi {

class IntegratorSymplecticEuler : public IntegratorBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using IntegratorBase::IntegratorBase;

  bool step(const Ref<const Vector<>> initial_state, const Scalar dt,
            Ref<Vector<>> final_state) const;
};

}  // namespace agi
