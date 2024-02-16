#include "agilib/simulator/model_body_drag_params.hpp"

namespace agi {

BodyDragParameters::BodyDragParameters()
  : cxy_(1.0), cz_(1.0), ax_(1.5e-2), ay_(1.5e-2), az_(3.0e-2), rho_(1.204) {}


bool BodyDragParameters::load(const Yaml& node) {
  if (node.isNull()) return false;

  cxy_ = node["horizontal_drag_coefficient"].as<Scalar>();
  cz_ = node["vertical_drag_coefficient"].as<Scalar>();
  ax_ = node["frontarea_x"].as<Scalar>();
  ay_ = node["frontarea_y"].as<Scalar>();
  az_ = node["frontarea_z"].as<Scalar>();
  rho_ = node["air_density"].as<Scalar>();

  return valid();
}

bool BodyDragParameters::valid() const {
  bool check = true;
  check &= cxy_ > 0;
  check &= cz_ > 0;
  check &= ax_ > 0;
  check &= ay_ > 0;
  check &= az_ > 0;
  check &= rho_ > 0;
  return check;
}

}  // namespace agi
