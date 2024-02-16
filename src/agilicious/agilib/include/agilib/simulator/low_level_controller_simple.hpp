#pragma once

#include <iostream>

#include "agilib/simulator/low_level_controller_base.hpp"
#include "agilib/simulator/low_level_controller_simple_params.hpp"
#include "agilib/utils/yaml.hpp"

namespace agi {

class LowLevelControllerSimple : public LowLevelControllerBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LowLevelControllerSimple(Quadrotor quad, const Scalar ctrl_dt,
                           const LowLevelControllerSimpleParams& params,
                           const std::string& name = "SimpleLLC");
  bool setCommand(const Command& cmd) override;
  void run() override;
  bool updateQuad(const Quadrotor& quad) override;

 private:
  // Quadrotor properties
  Matrix<4, 4> B_allocation_;
  Matrix<4, 4> B_allocation_inv_;
  LowLevelControllerSimpleParams params_;
};


}  // namespace agi
