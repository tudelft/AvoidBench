#pragma once

#include "agilib/base/module.hpp"
#include "agilib/base/parameter_base.hpp"
#include "agilib/math/math.hpp"
#include "agilib/math/types.hpp"
#include "agilib/types/command.hpp"
#include "agilib/types/quad_state.hpp"
#include "agilib/types/quadrotor.hpp"

namespace agi {

class LowLevelControllerBase : public Module<LowLevelControllerBase> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  LowLevelControllerBase(const Quadrotor& quad, const Scalar ctrl_dt,
                         const std::string& name = "LowLevelControllerBase");
  ~LowLevelControllerBase() override = default;
  virtual bool updateQuad(const Quadrotor&);
  virtual bool setState(const QuadState& state);
  virtual bool setCommand(const Command& cmd);
  virtual bool getMotorCommand(Ref<Vector<4>> motors);
  virtual bool setParamDir(const fs::path& param_dir);


 protected:
  // Command
  Command cmd_;

  // State of Quadrotor
  QuadState state_;

  // Control Frequency
  const float ctrl_dt_;

  // Motor speeds calculated by the controller
  Vector<4> motor_omega_des_;

  // Quadcopter to which the controller is applied
  Quadrotor quad_;

  // Directory to load parameters or thrust maps
  fs::path param_dir_;

  // Method that runs controller
  virtual void run() = 0;
};


}  // namespace agi
