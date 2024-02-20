#pragma once

#include "agilib/math/integrator_base.hpp"

namespace agi {

class IntegratorEuler : public IntegratorBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  using IntegratorBase::IntegratorBase;

  bool step(const Ref<const Vector<>> initial_state, const Scalar dt,
            Ref<Vector<>> final_state) const;
};

}  // namespace agi
