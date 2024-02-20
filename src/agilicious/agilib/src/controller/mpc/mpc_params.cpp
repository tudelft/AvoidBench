#include "agilib/controller/mpc/mpc_params.hpp"

namespace agi {

MpcParameters::MpcParameters()
  : threaded_preparation_(false),
    timing_{true},
    max_wait_time_before_preparation_{0.1},
    max_wait_time_after_preparation_{0.01},
    Q_pos_(200, 200, 500),
    Q_att_(5, 5, 200),
    Q_vel_(Vector<3>::Constant(1)),
    Q_omega_xy_(Vector<2>::Constant(1)),
    Q_omega_z_(1),
    R_(Vector<4>::Constant(6)),
    exp_decay_(1.0),
    cog_enable_(false),
    cog_abs_velocity_limit_(0.25),
    cog_abs_omega_limit_(0.5),
    cog_rel_hover_thrust_limit_(0.5),
    cog_height_limit_(0.5),
    Q_cog_omega_(Vector<2>::Constant(1e-0)),
    Q_cog_lengths_(Vector<2>::Constant(1e-9)),
    Q_cog_omega_init_(Vector<2>::Constant(1e+2)),
    Q_cog_lengths_init_(Vector<2>::Constant(1e-10)),
    R_cog_(Vector<2>::Constant(1e-1)) {}

bool MpcParameters::load(const Yaml& node) {
  if (node.isNull()) {
    return false;
  }

  threaded_preparation_ = node["threaded_preparation"].as<bool>();
  timing_ = node["timing"].as<bool>();

  Q_pos_(0) = node["Q_pos_x"].as<Scalar>();
  Q_pos_(1) = node["Q_pos_y"].as<Scalar>();
  Q_pos_(2) = node["Q_pos_z"].as<Scalar>();
  Q_att_(0) = node["Q_att_x"].as<Scalar>();
  Q_att_(1) = node["Q_att_y"].as<Scalar>();
  Q_att_(2) = node["Q_att_z"].as<Scalar>();
  node["Q_vel"] >> Q_vel_;
  node["Q_omega_xy"] >> Q_omega_xy_;
  Q_omega_z_ = node["Q_omega_z"].as<Scalar>();
  node["R"] >> R_;


  cog_enable_ = node["cog_enable"].as<bool>();
  cog_abs_velocity_limit_ = node["cog_abs_velocity_limit"].as<Scalar>();
  cog_abs_omega_limit_ = node["cog_abs_omega_limit"].as<Scalar>();
  cog_rel_hover_thrust_limit_ = node["cog_rel_hover_thrust_limit"].as<Scalar>();
  cog_height_limit_ = node["cog_height_limit"].as<Scalar>();
  Q_cog_omega_ = Vector<2>::Constant(node["Q_cog_omega"].as<Scalar>());
  Q_cog_lengths_ = Vector<2>::Constant(node["Q_cog_legnths"].as<Scalar>());
  Q_cog_omega_init_ =
    Vector<2>::Constant(node["Q_cog_omega_init"].as<Scalar>());
  Q_cog_lengths_init_ =
    Vector<2>::Constant(node["Q_cog_legnths_init"].as<Scalar>());
  R_cog_ = Vector<2>::Constant(node["R_cog"].as<Scalar>());

  exp_decay_ = node["exp_decay"].as<Scalar>();

  max_wait_time_before_preparation_ =
    node["max_wait_time_before_preparation"].as<Scalar>();
  max_wait_time_after_preparation_ =
    node["max_wait_time_after_preparation"].as<Scalar>();

  return valid();
}

bool MpcParameters::valid() const {
  bool check = true;
  check &= Q_pos_.allFinite() && (Q_pos_.array() >= 0.0).all();
  check &= Q_att_.allFinite() && (Q_att_.array() >= 0.0).all();
  check &= Q_vel_.allFinite() && (Q_vel_.array() >= 0.0).all();
  check &= Q_omega_xy_.allFinite() && (Q_omega_xy_.array() >= 0.0).all();
  check &= std::isfinite(Q_omega_z_);
  check &= R_.allFinite() && (R_.array() >= 0.0).all();
  check &= std::isfinite(exp_decay_);
  check &= max_wait_time_before_preparation_ >= 0.0;
  check &= max_wait_time_after_preparation_ >= 0.0;

  return check;
}

std::ostream& operator<<(std::ostream& os, const MpcParameters& params) {
  os << "MPC Parameters\n";
  os << "threaded_preparation:               " << params.threaded_preparation_
     << '\n';
  os << "timing:                             " << params.timing_ << '\n';
  os << "max_wait_time_before_preparation:   "
     << params.max_wait_time_before_preparation_ << '\n';
  os << "max_wait_time_after_preparation:    "
     << params.max_wait_time_after_preparation_ << '\n';
  os << "Q_pos:                              " << params.Q_pos_.transpose()
     << '\n';
  os << "Q_att:                              " << params.Q_att_.transpose()
     << '\n';
  os << "Q_vel:                              " << params.Q_vel_.transpose()
     << '\n';
  os << "Q_omega_xy:                         " << params.Q_omega_xy_.transpose()
     << '\n';
  os << "Q_omega_z:                          " << params.Q_omega_z_ << '\n';
  os << "R:                                  " << params.R_.transpose() << '\n';
  os << "exp_decay:                   " << params.exp_decay_;
  os << "cog_enable:                         " << params.cog_enable_ << '\n';
  os << "cog_abs_velocity_limit:             " << params.cog_abs_velocity_limit_
     << '\n';
  os << "cog_abs_omega_limit:                " << params.cog_abs_omega_limit_
     << '\n';
  os << "cog_rel_hover_thrust_limit:         "
     << params.cog_rel_hover_thrust_limit_ << '\n';
  os << "cog_height_limit_:                  " << params.cog_height_limit_
     << '\n';
  os << "Q_cog_omega:                        "
     << params.Q_cog_omega_.transpose() << '\n';
  os << "Q_cog_lengths:                      "
     << params.Q_cog_lengths_.transpose() << '\n';
  os << "Q_cog_omega_init:                   "
     << params.Q_cog_omega_init_.transpose() << '\n';
  os << "Q_cog_lengths_init:                 "
     << params.Q_cog_lengths_init_.transpose() << '\n';
  os << "R_cog:                              " << params.R_cog_.transpose()
     << '\n';

  return os;
}

}  // namespace agi
