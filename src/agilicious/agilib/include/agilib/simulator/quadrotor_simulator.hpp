#pragma once

// std
#include <memory>
#include <queue>
#include <type_traits>

// agilib
#include "agilib/base/module.hpp"
#include "agilib/math/integrator_symplectic_euler.hpp"
#include "agilib/math/types.hpp"
#include "agilib/simulator/low_level_controller_betaflight.hpp"
#include "agilib/simulator/low_level_controller_simple.hpp"
#include "agilib/simulator/model_base.hpp"
#include "agilib/simulator/model_init.hpp"
#include "agilib/simulator/model_motor.hpp"
#include "agilib/simulator/model_rigid_body.hpp"
#include "agilib/simulator/model_thrust_torque_simple.hpp"
#include "agilib/simulator/simulator_base.hpp"
#include "agilib/simulator/simulator_params.hpp"
#include "agilib/types/command.hpp"
#include "agilib/types/quad_state.hpp"
#include "agilib/types/quadrotor.hpp"

namespace agi {

class QuadrotorSimulator : public SimulatorBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  QuadrotorSimulator(const Quadrotor &quadrotor);
  QuadrotorSimulator(const SimulatorParams &params = SimulatorParams());

  // reset
  bool reset(const bool &reset_time = true) override;
  bool reset(const QuadState &state) override;

  // run the quadrotor simulator
  bool run(const Scalar ctl_dt) override;
  bool run(const Command &cmd, const Scalar ctl_dt);

  // public get functions
  bool getState(QuadState *const state) const override;
  bool getQuadrotor(Quadrotor *const quad) const;
  const Quadrotor &getQuadrotor() const;
  const std::shared_ptr<LowLevelControllerBase> getLowLevelController() const;

  // public set functions
  bool setState(const QuadState &state) override;
  bool setCommand(const Command &cmd);
  bool updateQuad(const Quadrotor &quad);
  void clearModelPipeline() { model_pipeline_.clear(); }


  // public pipeline construction
  template<class T>
  std::shared_ptr<T> addModel(const T &&mdl) {
    static_assert(std::is_base_of<ModelBase, T>::value,
                  "Model must be derived from ModelBase");
    model_pipeline_.push_back(std::make_shared<T>(mdl));
    return std::dynamic_pointer_cast<T>(model_pipeline_.back());
  }


 private:
  // quadrotor dynamics, integrator
  SimulatorParams params_;
  std::shared_ptr<LowLevelControllerBase> ctrl_;
  std::vector<std::shared_ptr<ModelBase>> model_pipeline_;
  std::shared_ptr<IntegratorBase> integrator_ptr_;


  // control command
  std::queue<Command> cmd_queue_;

  // quadrotor state
  QuadState state_;

  void updateState(const QuadState &, Scalar);
  DynamicsFunction getDynamics() const;
  bool computeDynamics(const Ref<const Vector<QS::SIZE>>,
                       Ref<Vector<QS::SIZE>>) const;
};


}  // namespace agi
