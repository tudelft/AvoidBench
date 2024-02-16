#pragma once

#include "agilib/base/parameter_base.hpp"

namespace agi {

struct MpcParameters : public ParameterBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MpcParameters();
  MpcParameters(const MpcParameters& rhs) = default;

  using ParameterBase::load;
  bool load(const Yaml& node) override;

  bool threaded_preparation_;
  bool timing_;

  Scalar max_wait_time_before_preparation_;
  Scalar max_wait_time_after_preparation_;

  Vector<3> Q_pos_;
  Vector<3> Q_att_;
  Vector<3> Q_vel_;
  Vector<2> Q_omega_xy_;
  Scalar Q_omega_z_;
  Vector<4> R_;

  Scalar exp_decay_;

  // CoG filter
  bool cog_enable_;
  Scalar cog_abs_velocity_limit_;
  Scalar cog_abs_omega_limit_;
  Scalar cog_rel_hover_thrust_limit_;
  Scalar cog_height_limit_;
  Vector<2> Q_cog_omega_;
  Vector<2> Q_cog_lengths_;
  Vector<2> Q_cog_omega_init_;
  Vector<2> Q_cog_lengths_init_;
  Vector<2> R_cog_;

  bool valid() const override;

  friend std::ostream& operator<<(std::ostream& os,
                                  const MpcParameters& params);
};

}  // namespace agi
