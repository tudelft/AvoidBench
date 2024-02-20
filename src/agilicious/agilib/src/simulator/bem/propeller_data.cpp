#include "agilib/simulator/bem/propeller_data.hpp"

namespace agi {

bool PropellerData::update() {
  if (!valid()) {
    return false;
  }
  prop_area_ = M_PI * r_prop_ * r_prop_;  // Propeller Area [m^2]
  e_ = ef_ * r_prop_;                     // Hinge offset [m]
  inertia_blade_ =
    mb_ * cg_ * cg_ + 0.25 * r_prop_ * r_prop_ / 12;  // Inertia of one blade
  moment_blade_ = mb_ * cg_ * G;  // Moment of blade around the hinge
  return true;
}


bool PropellerData::valid() const {
  bool check = true;
  check &= (rho_ > 0.0);
  check &= (r_prop_ > 0.0);
  check &= (ef_ >= 0.0) && (ef_ <= 1.0);
  check &= (sigma_ > 0.0) && (sigma_ < 1.0);
  check &= (theta0_ > 0.0) && (theta0_ < M_PI_2);
  check &= (std::fabs(theta1_) < M_PI_2);
  check &= (b_ > 0.0);
  check &= (mb_ > 0.0) && (mb_ < 1.0);
  check &= (cg_ > 0.0) && (cg_ < 1.0);
  check &= (cd_ >= 0.0);
  check &= (cl_ >= 0.0);
  check &= (ci_ >= 0.0) && (ci_ < 0.1);
  check &= (co_ >= 0.0) && (co_ < 0.1);
  check &= (k_spring_ >= 0.0);
  return check;
}


std::ostream& operator<<(std::ostream& os, const PropellerData& p) {
  os.precision(3);
  os << std::scientific;
  os << "PropellerData:\n"
     << "Air Density            " << p.rho_ << "\n"
     << "Radius                 " << p.r_prop_ << "\n"
     << "Propeller Area         " << p.prop_area_ << "\n"
     << "Relative hinge offset  " << p.ef_ << "\n"
     << "Hinge offset           " << p.e_ << "\n"
     << "Inner chord length     " << p.ci_ << "\n"
     << "Outer chord length     " << p.co_ << "\n"
     << "Solidity ratio         " << p.sigma_ << "\n"
     << "Pitch                  " << p.theta0_ << "\n"
     << "Twist                  " << p.theta1_ << "\n"
     << "Number of blades       " << p.b_ << "\n"
     << "Mass of one blade      " << p.mb_ << "\n"
     << "CoG Distance           " << p.cg_ << "\n"
     << "Drag coefficient       " << p.cd_ << "\n"
     << "Lift coefficient       " << p.cl_ << "\n"
     << "Spring constant        " << p.k_spring_ << std::endl;
  os.precision();
  os.unsetf(std::ios::scientific);
  return os;
}

}  // namespace agi
