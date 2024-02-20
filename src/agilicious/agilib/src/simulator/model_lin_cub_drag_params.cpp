#include "agilib/simulator/model_lin_cub_drag_params.hpp"

namespace agi {

LinCubDragParameters::LinCubDragParameters()
  : lin_drag_coeff_(Vector<3>::Zero()),
    cub_drag_coeff_(Vector<3>::Zero()),
    induced_lift_coeff_(0.0) {}


bool LinCubDragParameters::load(const Yaml& node) {
  if (node.isNull()) return false;

  node["lin_drag_coeff"] >> lin_drag_coeff_;
  node["cub_drag_coeff"] >> cub_drag_coeff_;
  induced_lift_coeff_ = node["induced_lift_coeff"].as<Scalar>();

  return valid();
}

bool LinCubDragParameters::valid() const {
  bool check = true;

  check &= lin_drag_coeff_.allFinite();
  check &= cub_drag_coeff_.allFinite();
  check &= induced_lift_coeff_ >= 0.0;

  return check;
}

}  // namespace agi
