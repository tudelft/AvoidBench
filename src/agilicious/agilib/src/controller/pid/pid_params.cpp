#include "agilib/controller/pid/pid_params.hpp"

#include <iostream>
namespace agi {

PidParameters::PidParameters()
  : kp_rate_(20.0, 20.0, 4.0),
    kd_rate_(0.2, 0.2, 0.0),
    ki_rate_(0.0, 0.0, 0.0),
    integration_max_(10.0, 10.0, 5.0),
    filter_sampling_frequency_(1000),
    filter_cutoff_frequency_(40) {}

bool PidParameters::load(const Yaml& node) {
  node["kp_rate"] >> kp_rate_;
  node["kd_rate"] >> kd_rate_;
  node["ki_rate"] >> ki_rate_;
  node["integration_max"] >> integration_max_;
  filter_sampling_frequency_ = node["filter_sampling_frequency"].as<Scalar>();
  filter_cutoff_frequency_ = node["filter_cutoff_frequency"].as<Scalar>();
  return valid();
}

bool PidParameters::valid() const {
  bool check = true;
  check &= (filter_sampling_frequency_ > 2.0 * filter_cutoff_frequency_);
  check &= (filter_cutoff_frequency_ > 0.0);
  check &= kp_rate_.allFinite() && (kp_rate_.array() >= 0.0).all();
  check &= kd_rate_.allFinite() && (kd_rate_.array() >= 0.0).all();
  check &= ki_rate_.allFinite() && (ki_rate_.array() >= 0.0).all();
  check &=
    integration_max_.allFinite() && (integration_max_.array() >= 0.0).all();

  return check;
}

}  // namespace agi
