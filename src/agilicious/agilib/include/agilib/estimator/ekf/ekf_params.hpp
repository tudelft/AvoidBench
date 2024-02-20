
#pragma once

#include "agilib/base/parameter_base.hpp"
#include "agilib/math/types.hpp"
#include "agilib/utils/logger.hpp"

namespace agi {

struct EkfParameters : public ParameterBase {
  EkfParameters();

  using ParameterBase::load;
  bool load(const Yaml& node) override;

  bool valid() const override;

  bool enable_timing;
  bool use_imu;

  Scalar max_update_wait_time;
  bool update_on_get;
  Scalar jump_pos_threshold;
  Scalar jump_att_threshold;

  Vector<3> R_pos;
  Vector<3> R_att;
  Vector<3> R_acc;
  Vector<3> R_omega;

  Vector<3> Q_pos;
  Vector<4> Q_att;
  Vector<3> Q_vel;
  Vector<3> Q_ome;
  Vector<3> Q_acc;
  Vector<3> Q_bome;
  Vector<3> Q_bacc;

  Vector<3> Q_init_pos;
  Vector<4> Q_init_att;
  Vector<3> Q_init_vel;
  Vector<3> Q_init_ome;
  Vector<3> Q_init_acc;
  Vector<3> Q_init_bome;
  Vector<3> Q_init_bacc;
};

}  // namespace agi
