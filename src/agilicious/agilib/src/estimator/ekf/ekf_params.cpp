#include "agilib/estimator/ekf/ekf_params.hpp"

namespace agi {

EkfParameters::EkfParameters()
  : enable_timing(true),
    use_imu(true),
    max_update_wait_time(0.050),
    update_on_get(false),
    jump_pos_threshold(0.0),
    jump_att_threshold(0.0 * M_PI),
    R_pos(Vector<3>::Constant(1e-6)),
    R_att(Vector<3>::Constant(1e-6)),
    R_acc(Vector<3>::Constant(1e-1)),
    R_omega(Vector<3>::Constant(1e-2)),
    Q_pos(Vector<3>::Constant(1e-6)),
    Q_att(Vector<4>::Constant(1e-6)),
    Q_vel(Vector<3>::Constant(1e-3)),
    Q_ome(Vector<3>::Constant(1e-3)),
    Q_acc(Vector<3>::Constant(1e-2)),
    Q_bome(Vector<3>::Constant(1e-9)),
    Q_bacc(Vector<3>::Constant(1e-9)),
    Q_init_pos(Vector<3>::Constant(1e-6)),
    Q_init_att(Vector<4>::Constant(1e-6)),
    Q_init_vel(Vector<3>::Constant(1e-3)),
    Q_init_ome(Vector<3>::Constant(1e-3)),
    Q_init_acc(Vector<3>::Constant(1e-3)),
    Q_init_bome(Vector<3>::Constant(1e-6)),
    Q_init_bacc(Vector<3>::Constant(1e-6)) {}


bool EkfParameters::load(const Yaml& node) {
  if (node.isNull()) return false;

  enable_timing = node["enable_timing"].as<bool>();
  use_imu = node["use_imu"].as<bool>();
  max_update_wait_time = node["max_update_wait_time"].as<Scalar>();
  update_on_get = node["update_on_get"].as<bool>();
  jump_pos_threshold = node["jump_pos_threshold"].as<Scalar>();
  jump_att_threshold = node["jump_att_threshold"].as<Scalar>();
  node["R_pos"] >> R_pos;
  node["R_att"] >> R_att;
  node["R_acc"] >> R_acc;
  node["R_omega"] >> R_omega;
  node["Q_pos"] >> Q_pos;
  node["Q_att"] >> Q_att;
  node["Q_vel"] >> Q_vel;
  node["Q_ome"] >> Q_ome;
  node["Q_acc"] >> Q_acc;
  node["Q_bome"] >> Q_bome;
  node["Q_bacc"] >> Q_bacc;
  node["Q_init_pos"] >> Q_init_pos;
  node["Q_init_att"] >> Q_init_att;
  node["Q_init_vel"] >> Q_init_vel;
  node["Q_init_ome"] >> Q_init_ome;
  node["Q_init_acc"] >> Q_init_acc;
  node["Q_init_bome"] >> Q_init_bome;
  node["Q_init_bacc"] >> Q_init_bacc;

  return valid();
}

bool EkfParameters::valid() const {
  bool check = true;

  check &= std::isfinite(max_update_wait_time);

  check &= R_pos.allFinite();
  check &= R_att.allFinite();
  check &= R_acc.allFinite();
  check &= R_omega.allFinite();

  check &= Q_pos.allFinite();
  check &= Q_att.allFinite();
  check &= Q_vel.allFinite();
  check &= Q_ome.allFinite();
  check &= Q_acc.allFinite();
  check &= Q_bome.allFinite();
  check &= Q_bacc.allFinite();

  check &= Q_init_pos.allFinite();
  check &= Q_init_att.allFinite();
  check &= Q_init_vel.allFinite();
  check &= Q_init_ome.allFinite();
  check &= Q_init_acc.allFinite();
  check &= Q_init_bome.allFinite();
  check &= Q_init_bacc.allFinite();

  return check;
}

}  // namespace agi
