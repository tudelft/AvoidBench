#pragma once

#include <cmath>
#include <mutex>
#include <thread>

#include "agilib/controller/controller_base.hpp"
#include "agilib/controller/indi/indi_params.hpp"
#include "agilib/math/gravity.hpp"
#include "agilib/math/math.hpp"
#include "agilib/math/types.hpp"
#include "agilib/types/command.hpp"
#include "agilib/types/imu_sample.hpp"
#include "agilib/types/quad_state.hpp"
#include "agilib/types/quadrotor.hpp"
#include "agilib/utils/logger.hpp"
#include "agilib/utils/low_pass_filter.hpp"
#include "agilib/utils/timer.hpp"

namespace agi {


class IndiController : public ControllerBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  IndiController(const Quadrotor& quad,
                 const std::shared_ptr<IndiParameters>& params);
  ~IndiController();

  virtual bool getCommand(const QuadState& state,
                          const SetpointVector& references,
                          SetpointVector* const setpoints) override;

  bool reset();
  bool reset(const QuadState& state);

  bool updateParameters(const Quadrotor& quad,
                        const std::shared_ptr<IndiParameters> params);
  bool updateParameters(const Quadrotor& params);
  bool updateParameters(const std::shared_ptr<IndiParameters> params);
  std::shared_ptr<IndiParameters> getParameters();

  void addImuSample(const ImuSample& sample) override { imu_ = sample; }

 private:
  Quadrotor quad_;
  std::shared_ptr<IndiParameters> params_;
  LowPassFilter<3> filterGyr_;
  LowPassFilter<4> filterMot_;

  ImuSample imu_;
  Matrix<> G_inv_;  // We know beforehand it's 4x4 matrix, but we need to do it
                    // dynamically allocated because the closed form inversion
                    // of a 4x4 matrix in Eigen seems broken
};

}  // namespace agi
