#include "agilib/estimator/ekf_imu/ekf_imu_params.hpp"

namespace agi {

EkfImuParameters::EkfImuParameters()
  : jump_pos_threshold(0.0),
    jump_att_threshold(0.0 * M_PI),
    R_pos(Vector<3>::Constant(1e-6)),
    R_att(Vector<3>::Constant(1e-6)),
    R_acc(Vector<3>::Constant(1e-1)),
    R_omega(Vector<3>::Constant(1e-2)),
    Q_pos(Vector<3>::Constant(1e-6)),
    Q_att(Vector<4>::Constant(1e-6)),
    Q_vel(Vector<3>::Constant(1e-3)),
    Q_bome(Vector<3>::Constant(1e-9)),
    Q_bacc(Vector<3>::Constant(1e-9)),
    Q_init_pos(Vector<3>::Constant(1e-6)),
    Q_init_att(Vector<4>::Constant(1e-6)),
    Q_init_vel(Vector<3>::Constant(1e-3)),
    Q_init_bome(Vector<3>::Constant(1e-6)),
    Q_init_bacc(Vector<3>::Constant(1e-6)),
    update_on_get(false) {}

bool EkfImuParameters::load(const Yaml& node) {
  if (node.isNull()) return false;

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
  node["Q_bome"] >> Q_bome;
  node["Q_bacc"] >> Q_bacc;
  node["Q_init_pos"] >> Q_init_pos;
  node["Q_init_att"] >> Q_init_att;
  node["Q_init_vel"] >> Q_init_vel;
  node["Q_init_bome"] >> Q_init_bome;
  node["Q_init_bacc"] >> Q_init_bacc;

  return valid();
}

bool EkfImuParameters::valid() const {
  bool check = true;

  check &= R_pos.allFinite();
  check &= R_att.allFinite();
  check &= R_acc.allFinite();
  check &= R_omega.allFinite();

  check &= Q_pos.allFinite();
  check &= Q_att.allFinite();
  check &= Q_vel.allFinite();
  check &= Q_bome.allFinite();
  check &= Q_bacc.allFinite();

  check &= Q_init_pos.allFinite();
  check &= Q_init_att.allFinite();
  check &= Q_init_vel.allFinite();
  check &= Q_init_bome.allFinite();
  check &= Q_init_bacc.allFinite();

  return check;
}

std::ostream& operator<<(std::ostream& os, const EkfImuParameters& params) {
  os << "EKF IMU Parameters:\n";

  os << "R_pos:       " << params.R_pos.transpose() << '\n';
  os << "R_att:       " << params.R_att.transpose() << '\n';
  os << "R_acc:       " << params.R_acc.transpose() << '\n';
  os << "R_omega:     " << params.R_omega.transpose() << '\n';
  os << "Q_pos:       " << params.Q_pos.transpose() << '\n';
  os << "Q_att:       " << params.Q_att.transpose() << '\n';
  os << "Q_vel:       " << params.Q_vel.transpose() << '\n';
  os << "Q_bome:      " << params.Q_bome.transpose() << '\n';
  os << "Q_bacc:      " << params.Q_bacc.transpose() << '\n';
  os << "Q_init_pos:  " << params.Q_init_pos.transpose() << '\n';
  os << "Q_init_att:  " << params.Q_init_att.transpose() << '\n';
  os << "Q_init_vel:  " << params.Q_init_vel.transpose() << '\n';
  os << "Q_init_bome: " << params.Q_init_bome.transpose() << '\n';
  os << "Q_init_bacc: " << params.Q_init_bacc.transpose() << '\n';

  os << "update_on_get:       " << params.update_on_get << '\n';
  os << "jump_pos_threshold:  " << params.jump_pos_threshold << '\n';
  os << "jump_att_threshold:  " << params.jump_att_threshold << '\n';

  return os;
}

}  // namespace agi
