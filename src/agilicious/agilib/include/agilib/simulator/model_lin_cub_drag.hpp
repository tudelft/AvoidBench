#pragma once

#include "agilib/math/math.hpp"
#include "agilib/simulator/model_base.hpp"
#include "agilib/simulator/model_lin_cub_drag_params.hpp"

namespace agi {

class ModelLinCubDrag : public ModelBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ModelLinCubDrag(Quadrotor quad,
                  const LinCubDragParameters& params = LinCubDragParameters());

  bool run(const Ref<const Vector<QS::SIZE>>,
           Ref<Vector<QS::SIZE>>) const override;

 private:
  LinCubDragParameters params_;
};


}  // namespace agi