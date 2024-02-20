#include "agilib/simulator/model_propeller_bem_params.hpp"

#include "agilib/math/math.hpp"

namespace agi {

BEMParameters::BEMParameters()
  : rho_(1.204),
    r_prop_(6.477e-2),
    ef_(0.1),
    sigma_(0.215),
    theta0_(0.40055306),
    theta1_(-0.139626),
    b_(3.0),
    ci_(1.7e-2),
    co_(0.8e-2),
    mb_(1.22e-3),
    cg_(2.7e-2),
    cd_(4.168863),
    cl_(4.797071),
    k_spring_(5.89) {
  prop_area_ = M_PI * r_prop_ * r_prop_;  // Propeller Area [m^2]
  e_ = ef_ * r_prop_;                     // Hinge offset [m]
}

bool BEMParameters::load(const Yaml& node) {
  if (node.isNull()) return false;

  rho_ = node["air_density"].as<Scalar>();
  r_prop_ = node["radius"].as<Scalar>();
  theta0_ = toRad(node["pitch"].as<Scalar>());
  theta1_ = toRad(node["twist"].as<Scalar>());
  sigma_ = node["solidity"].as<Scalar>();
  cg_ = node["distance_cog"].as<Scalar>();
  b_ = node["num_blades"].as<Scalar>();
  mb_ = node["mass_blade"].as<Scalar>();
  ef_ = node["rel_hinge_offset"].as<Scalar>();
  cd_ = node["drag_coefficient"].as<Scalar>();
  cl_ = node["lift_coefficient"].as<Scalar>();
  co_ = node["chord_outer"].as<Scalar>();
  ci_ = node["chord_inner"].as<Scalar>();
  k_spring_ = node["hinge_spring_constant"].as<Scalar>();

  return valid();
}

bool BEMParameters::valid() const {
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

std::ostream& operator<<(std::ostream& os, const BEMParameters& p) {
  os.precision(3);
  os << std::scientific;
  os << "BEMParameters:\n"
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
