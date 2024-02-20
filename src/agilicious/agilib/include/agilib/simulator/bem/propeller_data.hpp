#pragma once

#include <agilib/math/gravity.hpp>
#include <agilib/math/math.hpp>
#include <agilib/math/types.hpp>
#include <iostream>

namespace agi {


struct PropellerData {
  /* PARAMETER DECLARATIONS */
  Scalar rho_ = 1.204;                           // ISA air density [kg / m^3]
  Scalar r_prop_ = 5.1 * 2.54 / 2.0 * 1e-02;     // Propeller Radius in [m]
  Scalar prop_area_ = M_PI * r_prop_ * r_prop_;  // Propeller Area [m^2]
  Scalar ef_ = 0.1;                              // Hinge offset (relative)
  Scalar e_ = ef_ * r_prop_;                     // Hinge offset [m]
  Scalar sigma_ = 0.215;                         // Propeller Solidity Ratio
  Scalar theta0_ = toRad(22.95);                 // Propeller Pitch [rad]
  Scalar theta1_ = toRad(-8.0);                  // Propeller Twist [rad/m]
  Scalar b_ = 3;                                 // Number of Propeller Blades
  Scalar ci_ = 1.7 * 1e-02;                      // Chord length [m]
  Scalar co_ = 0.8 * 1e-02;                      // Chord length [m]
  Scalar mb_ = 1.22e-3;                          // Mass of one blade [kg]
  Scalar cg_ = 2.7e-3;  // Distance of prop CoG to shaft
  Scalar inertia_blade_ =
    mb_ * cg_ * cg_ + 0.25 * r_prop_ * r_prop_ / 12.0;  // Inertia of one blade
  Scalar moment_blade_ = mb_ * cg_ * G;  // Moment of blade around the hinge
  Scalar cd_ = 4.168863;                 // Constant drag coefficient
  Scalar cl_ = 4.797071;                 // Slope of the lift curve in [1/rad]
  Scalar k_spring_ = 5.89;               // Spring constant of the hinge spring

  bool update();
  bool valid() const;
  friend std::ostream& operator<<(std::ostream& os, const PropellerData& p);
};

}  // namespace agi
