#pragma once

#include <cmath>
#include <mutex>
#include <thread>

#include "agilib/controller/controller_base.hpp"
#include "agilib/controller/geometric/geo_params.hpp"
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


class GeometricController : public ControllerBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GeometricController(const Quadrotor& quad,
                      const std::shared_ptr<GeometricControllerParams>& params);
  ~GeometricController();

  virtual bool getCommand(const QuadState& state,
                          const SetpointVector& references,
                          SetpointVector* const setpoints) override;

  bool updateParameters(
    const Quadrotor& quad,
    const std::shared_ptr<GeometricControllerParams> params);
  bool updateParameters(const Quadrotor& params);
  bool updateParameters(
    const std::shared_ptr<GeometricControllerParams> params);
  std::shared_ptr<GeometricControllerParams> getParameters();

  bool addImu(const ImuSample& imu);

 private:
  Vector<3> tiltPrioritizedControl(const Quaternion& q,
                                   const Quaternion& q_des);
  Vector<4> qTimeDerivative(const Quaternion& q, const Scalar& t);

  Quadrotor quad_;
  std::shared_ptr<GeometricControllerParams> params_;
  LowPassFilter<3> filterAcc_;
  LowPassFilter<4> filterMot_;

  ImuSample imu_;
};

}  // namespace agi
