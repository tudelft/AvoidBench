#pragma once

#include "agilib/base/module.hpp"
#include "agilib/base/parameter_base.hpp"
#include "agilib/math/integrator_base.hpp"
#include "agilib/simulator/low_level_controller_base.hpp"
#include "agilib/simulator/model_base.hpp"
#include "agilib/types/quadrotor.hpp"
#include "agilib/utils/module_config.hpp"

namespace agi {
class SimulatorParams : public ParameterBase {
 public:
  SimulatorParams(const Quadrotor& quadrotor = Quadrotor());
  SimulatorParams(const fs::path& filename,
                  const fs::path& agi_param_directory = "",
                  const fs::path& ros_param_directory = "");

  using ParameterBase::load;
  bool load(const Yaml& node) override;

  bool valid() const override;

  bool createModelPipeline(
    std::vector<std::shared_ptr<ModelBase>>& model_pipeline) const;
  bool createLowLevelController(
    std::shared_ptr<LowLevelControllerBase>& ctrl) const;
  bool createIntegrator(std::shared_ptr<IntegratorBase>& integrator,
                        DynamicsFunction dynamics_function) const;

  fs::path agi_param_directory_;
  fs::path ros_param_directory_;

  std::string controller_name_;
  fs::path controller_file_;

  std::string integrator_name_;
  Scalar sim_dt_;
  Scalar delay_;

  fs::path quad_file_;
  Quadrotor quadrotor_;

 private:
  Logger logger_{"SimulatorParams"};
  std::vector<ModuleConfig> model_configs_;
};
}  // namespace agi
