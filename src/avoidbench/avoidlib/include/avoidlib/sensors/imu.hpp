#pragma once

#include "avoidlib/common/types.hpp"
#include "avoidlib/sensors/sensor_base.hpp"

namespace avoidlib {

class IMU : SensorBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  IMU();
  ~IMU();

 private:
};
}  // namespace avoidlib
