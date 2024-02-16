#pragma once

#include "agilib/math/fast_atan2.hpp"
#include "agilib/math/math.hpp"
#include "agilib/math/types.hpp"
#include "agilib/simulator/bem/propeller_data.hpp"
#include "agilib/simulator/model_propeller_bem_params.hpp"
#include "agilib/types/quad_state.hpp"

namespace agi {

struct PropellerState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PropellerState(const BEMParameters& params);

  BEMParameters param_;
  ArrayVector<3> flu_conv_frd_{1, -1, -1};
  ArrayVector<QS::NMOT> omega_mot_;
  Vector<3> vel_;
  Vector<3> w_;
  Matrix<3, 3> rot_;
  Matrix<3, QS::NMOT> velocity_;

  Vector<QS::NMOT> vhor_;
  Vector<QS::NMOT> vver_;
  Vector<QS::NMOT> vtot_;
  ArrayVector<QS::NMOT> alpha_s_;
  Vector<QS::NMOT> mu_;
  Vector<QS::NMOT> K_;

  // to be calculated
  Vector<QS::NMOT> vind_;         // induced velocity
  Vector<QS::NMOT> a0_;           // coning angle
  Vector<QS::NMOT> a1s_;          // longitudinal flapping angle
  Vector<QS::NMOT> b1s_;          // lateral flapping angle
  ArrayVector<QS::NMOT> thrust_;  // thrust of the propeller
  ArrayVector<QS::NMOT> torque_;  // drag torque of the propeller
  ArrayVector<QS::NMOT> hforce_;  // h-force of the propeller

  void update(const Ref<const ArrayVector<QS::SIZE>>& state,
              const Ref<const Matrix<3, 4>>& t_BM);

  void calculateFlapping();
  friend std::ostream& operator<<(std::ostream& os, const PropellerState& s);
};

}  // namespace agi
