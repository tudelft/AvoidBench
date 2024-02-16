#include "agilib/sampler/position_based/position_sampler_params.hpp"

namespace agi {

bool PositionSamplerParameters::load(const Yaml& node) {
  if (node.isNull()) {
    return false;
  }
  node["axis_weights"] >> axis_weights_sqrt_;
  axis_weights_sqrt_ = axis_weights_sqrt_.array().sqrt();
  search_dt_ = node["search_dt"].as<Scalar>();
  search_tol_ = node["search_tol"].as<Scalar>();

  return valid();
}

bool PositionSamplerParameters::valid() const {
  bool check = true;
  check &=
    axis_weights_sqrt_.allFinite() && (axis_weights_sqrt_.array() >= 0.0).all();
  check &= search_tol_ >= 0.0;
  check &= search_dt_ > 0.0;

  return check;
}

}  // namespace agi
