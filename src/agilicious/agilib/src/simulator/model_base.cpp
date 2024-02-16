#include "agilib/simulator/model_base.hpp"

namespace agi {

ModelBase::ModelBase(Quadrotor& quad) : quad_(quad) {}

bool ModelBase::updateQuad(const Quadrotor& quad) {
  if (!quad.valid()) return false;
  quad_ = quad;
  return true;
}

}  // namespace agi
