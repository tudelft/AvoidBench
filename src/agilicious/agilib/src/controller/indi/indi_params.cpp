#include "agilib/controller/indi/indi_params.hpp"

#include <iostream>

namespace agi {

IndiParameters::IndiParameters()
  : filter_sampling_frequency_(100),
    filter_cutoff_frequency_(20),
    use_indi_(true) {}

bool IndiParameters::load(const Yaml& node) {
  filter_sampling_frequency_ = node["filter_sampling_frequency"].as<Scalar>();
  filter_cutoff_frequency_ = node["filter_cutoff_frequency"].as<Scalar>();
  use_indi_ = node["use_indi"].as<bool>();

  return valid();
}

bool IndiParameters::valid() const {
  bool check = true;
  check &= (filter_sampling_frequency_ > 2.0 * filter_cutoff_frequency_);
  check &= (filter_cutoff_frequency_ > 0.0);

  return check;
}

std::ostream& operator<<(std::ostream& os, const IndiParameters& params) {
  os << "IndiParameters:\n";
  os << "use_indi:                   " << params.use_indi_ << '\n';
  os << "kp_rate:                    " << params.kp_rate_.transpose() << '\n';
  os << "filter_sampling_frequency:  " << params.filter_sampling_frequency_
     << '\n';
  os << "filter_cutoff_frequency:    " << params.filter_cutoff_frequency_
     << '\n';
  return os;
}

}  // namespace agi
