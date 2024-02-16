#pragma once

#include "agilib/base/parameter_base.hpp"
#include "agilib/math/types.hpp"
#include "agilib/utils/logger.hpp"

namespace agi {

struct BEMParameters : public ParameterBase {
  BEMParameters();

  using ParameterBase::load;
  bool load(const Yaml& node) override;

  bool valid() const override;
  friend std::ostream& operator<<(std::ostream& os, const BEMParameters& p);


  Scalar rho_;        // ISA air density [kg / m^3]
  Scalar r_prop_;     // Propeller Radius in [m]
  Scalar prop_area_;  // Propeller Area [m^2]
  Scalar ef_;         // Hinge offset (relative)
  Scalar e_;          // Hinge offset [m]
  Scalar sigma_;      // Propeller Solidity Ratio
  Scalar theta0_;     // Propeller Pitch [rad]
  Scalar theta1_;     // Propeller Twist [rad/m]
  Scalar b_;          // Number of Propeller Blades
  Scalar ci_;         // Chord length [m]
  Scalar co_;         // Chord length [m]
  Scalar mb_;         // Mass of one blade [kg]
  Scalar cg_;         // Distance of prop CoG to shaft
  Scalar cd_;         // Constant drag coefficient
  Scalar cl_;         // Slope of the lift curve in [1/rad]
  Scalar k_spring_;   // Spring constant of the hinge spring
};

}  // namespace agi
