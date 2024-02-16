#pragma once

#include "agilib/base/parameter_base.hpp"
#include "agilib/math/types.hpp"

namespace agi {

struct MockVioParams : public ParameterBase {
  MockVioParams();

  using ParameterBase::load;
  bool load(const Yaml& node) override;

  bool valid() const override;

  bool use_imu;
  Scalar rise;

  Scalar image_dt;
  Scalar vio_dt;
  Scalar vio_latency;

  Vector<3> vio_pos_static_drift;
  Vector<3> vio_pos_dynamic_drift;
  Vector<3> vio_pos_noise;
  Vector<3> vio_att_noise;
  Vector<3> vio_vel_noise;
  Vector<3> vio_omega_bias_noise;
  Vector<3> vio_acc_bias_noise;

  Scalar imu_dt;
  Vector<3> imu_omega_noise;
  Vector<3> imu_acc_noise;
  Vector<3> imu_omega_bias;
  Vector<3> imu_acc_bias;
};

}  // namespace agi
