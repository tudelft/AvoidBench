#pragma once

#include <string>

#include "agilib/math/types.hpp"
#include "agilib/types/quad_state.hpp"
#include "agilib/types/quadrotor.hpp"
#include "agilib/utils/yaml.hpp"


namespace agi {

class ModelBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ModelBase(Quadrotor& quad);
  virtual ~ModelBase() = default;

  virtual bool updateQuad(const Quadrotor&);

  // IN: State x
  // IN-OUT: Derivative of State
  virtual bool run(const Ref<const Vector<QS::SIZE>>,
                   Ref<Vector<QS::SIZE>>) const = 0;

 protected:
  Quadrotor quad_;
};

}  // namespace agi
