#pragma once

#include "agilib/math/math.hpp"
#include "agilib/simulator/model_base.hpp"
#include "agilib/simulator/model_body_drag_params.hpp"

namespace agi {

class ModelBodyDrag : public ModelBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ModelBodyDrag(Quadrotor quad,
                const BodyDragParameters& params = BodyDragParameters());

  bool run(const Ref<const Vector<QS::SIZE>>,
           Ref<Vector<QS::SIZE>>) const override;

 private:
  BodyDragParameters params_;
};


}  // namespace agi
