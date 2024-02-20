#pragma once

#include <algorithm>
#include <fstream>
#include <iostream>

#include "agilib/simulator/low_level_controller_base.hpp"
#include "agilib/simulator/low_level_controller_betaflight_params.hpp"
#include "agilib/utils/low_pass_filter.hpp"

namespace agi {

class LowLevelControllerBetaflight : public LowLevelControllerBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LowLevelControllerBetaflight(Quadrotor quad, const Scalar ctrl_dt,
                               const LowLevelControllerBetaflightParams& params,
                               const std::string& name = "BetaFlight");
  bool setCommand(const Command& cmd) override;
  bool setState(const QuadState& state) override;
  void run() override;
  bool updateQuad(const Quadrotor& quad) override;

 private:
  // Quadrotor properties: allocation in BFL style
  // 1, -1,  1,  1,
  // 1, -1, -1, -1,
  // 1,  1,  1, -1,
  // 1,  1, -1,  1
  const Matrix<4, 4> B_allocation_ =
    (Matrix<4, 4>() << 1, -1, -1, -1, 1, 1, 1, -1, 1, -1, 1, 1, 1, 1, -1, 1)
      .finished();

  // Battery Voltage: fixed for now, add voltage model in the future
  Scalar voltage_ = 15.0;

  std::shared_ptr<LowPassFilter<3>> gyro_lpf_;
  std::shared_ptr<LowPassFilter<3>> dterm_lpf_;
  ArrayVector<3> error_integral_;
  LowLevelControllerBetaflightParams params_;
};


}  // namespace agi
