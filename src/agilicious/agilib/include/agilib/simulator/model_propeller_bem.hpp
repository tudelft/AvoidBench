#pragma once

#include <unistd.h>

#include <cmath>
#include <functional>

// Agilib
#include "agilib/math/gravity.hpp"
#include "agilib/math/math.hpp"
#include "agilib/simulator/model_base.hpp"
#include "agilib/utils/logger.hpp"
#include "agilib/utils/timer.hpp"

// BEM
#include "agilib/simulator/bem/brent.hpp"
#include "agilib/simulator/bem/functions.hpp"
#include "agilib/simulator/bem/gauss_kronrod.hpp"
#include "agilib/simulator/bem/propeller_data.hpp"
#include "agilib/simulator/bem/propeller_state.hpp"

namespace agi {


/* Note: this needs to be in the pipeline after the motor model ran. The angular
 * acceleration of the motors is actually used to calculate the gyroscopic
 * forces of the propeller.
 */
class ModelPropellerBEM : public ModelBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  ModelPropellerBEM(Quadrotor quad,
                    const BEMParameters& params = BEMParameters());
  ~ModelPropellerBEM() override;

  bool run(const Ref<const Vector<QS::SIZE>>,
           Ref<Vector<QS::SIZE>>) const override;

 private:
  // due to frame obscurring parts of the area below
  const Scalar thrust_scale_ = 0.9575;
  const GaussKronrod<GaussKronrodFunction<QS::NMOT>> gk_;

  mutable std::shared_ptr<PropellerState> prop_state_;
  mutable std::shared_ptr<IntegrandPsi> f_psi_thrust_;
  mutable std::shared_ptr<IntegrandPsi> f_psi_torque_;
  mutable std::shared_ptr<IntegrandPsi> f_psi_hforce_;
  mutable std::shared_ptr<IntegrandR> f_r_thrust_;
  mutable std::shared_ptr<IntegrandR> f_r_torque_;
  mutable std::shared_ptr<IntegrandR> f_r_hforce_;
  mutable std::shared_ptr<ThrustFunction> f_v1_;
  mutable ArrayVector<QS::NMOT> vind_;    // induced velocity
  mutable ArrayVector<QS::NMOT> vind_h_;  // vortex ring state fit
  Brent<BrentFunction<QS::NMOT>> vind_solver_;

  Logger logger_{"BEM Model"};
  mutable Timer timer_{"BEM Model"};
};


}  // namespace agi
