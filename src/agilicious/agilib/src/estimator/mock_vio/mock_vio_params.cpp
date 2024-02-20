#include "agilib/estimator/mock_vio/mock_vio_params.hpp"

namespace agi {

MockVioParams::MockVioParams()
  : use_imu(true),
    rise(0.0),
    image_dt(0.0),
    vio_dt(1.0 / 30.0),
    vio_latency(vio_dt),
    vio_pos_static_drift(Vector<3>::Zero()),
    vio_pos_dynamic_drift(Vector<3>::Zero()),
    vio_pos_noise(Vector<3>::Zero()),
    vio_att_noise(Vector<3>::Zero()),
    vio_vel_noise(Vector<3>::Zero()),
    vio_omega_bias_noise(Vector<3>::Zero()),
    vio_acc_bias_noise(Vector<3>::Zero()),
    imu_dt(0.0),
    imu_omega_noise(Vector<3>::Zero()),
    imu_acc_noise(Vector<3>::Zero()),
    imu_omega_bias(Vector<3>::Zero()),
    imu_acc_bias(Vector<3>::Zero()) {}


bool MockVioParams::load(const Yaml& node) {
  if (node.isNull()) return false;

  use_imu = node["use_imu"].as<bool>();
  vio_dt = node["vio_dt"].as<Scalar>();

  vio_latency = vio_dt;
  node["vio_latency"].getIfDefined(vio_latency);

  node["rise"].getIfDefined(rise);
  node["image_dt"].getIfDefined(image_dt);

  node["vio_pos_static_drift"].getIfDefined(vio_pos_static_drift);
  node["vio_pos_dynamic_drift"].getIfDefined(vio_pos_dynamic_drift);
  node["vio_pos_noise"].getIfDefined(vio_pos_noise);
  node["vio_vel_noise"].getIfDefined(vio_vel_noise);
  node["vio_att_noise"].getIfDefined(vio_att_noise);
  node["vio_omega_bias_noise"].getIfDefined(vio_omega_bias_noise);
  node["vio_acc_bias_noise"].getIfDefined(vio_acc_bias_noise);

  node["imu_dt"].getIfDefined(imu_dt);
  node["imu_omega_noise"].getIfDefined(imu_omega_noise);
  node["imu_acc_noise"].getIfDefined(imu_acc_noise);
  node["imu_omega_bias"].getIfDefined(imu_omega_bias);
  node["imu_acc_bias"].getIfDefined(imu_acc_bias);

  return valid();
}

bool MockVioParams::valid() const {
  bool check = true;

  check &= rise >= 0.0;
  check &= image_dt >= 0.0;
  check &= vio_dt > 0.0;
  check &= vio_latency >= 0.0;

  check &= vio_pos_static_drift.allFinite() &&
           (vio_pos_static_drift.array() >= 0.0).all();
  check &= vio_pos_dynamic_drift.allFinite() &&
           (vio_pos_dynamic_drift.array() >= 0.0).all();
  check &= vio_pos_noise.allFinite() && (vio_pos_noise.array() >= 0.0).all();
  check &= vio_att_noise.allFinite() && (vio_att_noise.array() >= 0.0).all();
  check &= vio_vel_noise.allFinite() && (vio_vel_noise.array() >= 0.0).all();
  check &= vio_omega_bias_noise.allFinite() &&
           (vio_omega_bias_noise.array() >= 0.0).all();
  check &=
    vio_acc_bias_noise.allFinite() && (vio_acc_bias_noise.array() >= 0.0).all();

  check &= imu_dt >= 0.0;
  check &=
    imu_omega_noise.allFinite() && (imu_omega_noise.array() >= 0.0).all();
  check &= imu_acc_noise.allFinite() && (imu_acc_noise.array() >= 0.0).all();
  check &= imu_omega_bias.allFinite() && (imu_omega_bias.array() >= 0.0).all();
  check &= imu_acc_bias.allFinite() && (imu_acc_bias.array() >= 0.0).all();

  return check;
}

}  // namespace agi
