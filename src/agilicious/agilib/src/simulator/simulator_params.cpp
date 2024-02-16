#include "agilib/simulator/simulator_params.hpp"

#include <unistd.h>

#include "agilib/math/integrator_euler.hpp"
#include "agilib/math/integrator_rk4.hpp"
#include "agilib/math/integrator_symplectic_euler.hpp"
#include "agilib/simulator/low_level_controller_betaflight.hpp"
#include "agilib/simulator/low_level_controller_simple.hpp"
#include "agilib/simulator/model_body_drag.hpp"
#include "agilib/simulator/model_init.hpp"
#include "agilib/simulator/model_lin_cub_drag.hpp"
#include "agilib/simulator/model_motor.hpp"
#include "agilib/simulator/model_propeller_bem.hpp"
#include "agilib/simulator/model_rigid_body.hpp"
#include "agilib/simulator/model_thrust_torque_simple.hpp"
#include "agilib/types/quadrotor.hpp"
#include "agilib/utils/file_utils.hpp"
#include "agilib/utils/module_config.hpp"

namespace agi {

SimulatorParams::SimulatorParams(const Quadrotor& quadrotor)
  : sim_dt_(0.001), delay_(0.0), quadrotor_(quadrotor) {
  // default simulator config
}

SimulatorParams::SimulatorParams(const fs::path& filename,
                                 const fs::path& agi_param_directory,
                                 const fs::path& ros_param_directory)
  : agi_param_directory_(agi_param_directory),
    ros_param_directory_(ros_param_directory),
    quad_file_(fs::path(ros_param_directory) / "quads" / "") {
  fs::path full_filename = filename;
  if (!checkFile(agi_param_directory_, &full_filename))
    throw ParameterException("Simulation config file not found!\n" +
                             filename.string() + "\n" + full_filename.string());

  if (!ParameterBase::load(full_filename))
    throw ParameterException(full_filename);
}
bool SimulatorParams::load(const Yaml& yaml) {
  if (agi_param_directory_.empty()) {
    static constexpr int PATH_LEN = 2048;
    char path_cstr[PATH_LEN];
    if (readlink("/proc/self/exe", path_cstr, PATH_LEN)) {
      const fs::path agidir(path_cstr);

      fs::path::iterator start_it = agidir.begin();
      fs::path::iterator end_it =
        std::lower_bound(start_it, agidir.end(), std::string("agilicious"));

      if (end_it == agidir.end()) {
        throw ParameterException(
          "No directory provided and agilicious directory not found!");
      } else {
        for (fs::path::iterator it = agidir.begin(); it != end_it; it++)
          agi_param_directory_ /= *it;
        agi_param_directory_ =
          agi_param_directory_ / "agilicious" / "agilib" / "params";
      }
    }
  }

  // Quadrotor
  quad_file_ = getQuadFile(yaml, quad_file_, agi_param_directory_);

  if (!quadrotor_.load(quad_file_) || !quadrotor_.valid())
    throw ParameterException("Could not load Quadrotor parameters from: " +
                             quad_file_.string());

  delay_ = yaml["delay"].as<Scalar>();
  controller_name_ = yaml["controller"]["name"].as<std::string>();
  controller_file_ = yaml["controller"]["file"].as<std::string>();
  integrator_name_ = yaml["integrator"]["name"].as<std::string>();
  sim_dt_ = yaml["integrator"]["step_size"].as<Scalar>();

  const int num_models = yaml["model_pipeline"].size();
  model_configs_.clear();
  for (int i = 0; i < num_models; i++) {
    ModuleConfig model_config;
    model_config.loadIfUndefined(yaml["model_pipeline"][i]);
    model_configs_.push_back(model_config);
  }

  return valid();
}

bool SimulatorParams::valid() const {
  bool check = true;

  check &= std::isfinite(delay_) && delay_ >= 0.0;
  check &= quadrotor_.valid();

  return check;
}

bool SimulatorParams::createModelPipeline(
  std::vector<std::shared_ptr<ModelBase>>& model_pipeline) const {
  logger_.info("Creating model pipeline!");

  model_pipeline.clear();

  if (model_configs_.empty()) {
    // default dynamics simulation
    model_pipeline.emplace_back(
      std::make_shared<ModelInit>(ModelInit{quadrotor_}));
    model_pipeline.emplace_back(
      std::make_shared<ModelMotor>(ModelMotor{quadrotor_}));
    model_pipeline.emplace_back(std::make_shared<ModelThrustTorqueSimple>(
      ModelThrustTorqueSimple{quadrotor_}));
    model_pipeline.emplace_back(
      std::make_shared<ModelRigidBody>(ModelRigidBody{quadrotor_}));
    return true;
  }

  for (auto model_config : model_configs_) {
    logger_.info("Loading dynamics model [%s] from [%s].",
                 model_config.type.c_str(), model_config.file.string().c_str());

    if (model_config.type == "ModelInit") {
      model_pipeline.emplace_back(
        std::make_shared<ModelInit>(ModelInit{quadrotor_}));
    } else if (model_config.type == "ModelMotor") {
      model_pipeline.emplace_back(
        std::make_shared<ModelMotor>(ModelMotor{quadrotor_}));
    } else if (model_config.type == "ModelRotorSimple") {
      model_pipeline.emplace_back(std::make_shared<ModelThrustTorqueSimple>(
        ModelThrustTorqueSimple{quadrotor_}));
    } else if (model_config.type == "ModelRotorBEM") {
      BEMParameters bem_params;
      bem_params.load(agi_param_directory_ / "dynamics_models" /
                      model_config.file);
      model_pipeline.emplace_back(std::make_shared<ModelPropellerBEM>(
        ModelPropellerBEM{quadrotor_, bem_params}));
    } else if (model_config.type == "ModelBodyDrag") {
      BodyDragParameters body_drag_params;
      body_drag_params.load(agi_param_directory_ / "dynamics_models" /
                            model_config.file);
      model_pipeline.emplace_back(std::make_shared<ModelBodyDrag>(
        ModelBodyDrag{quadrotor_, body_drag_params}));
    } else if (model_config.type == "ModelLinCubDragIndLift") {
      LinCubDragParameters lin_cub_drag_params;
      lin_cub_drag_params.load(agi_param_directory_ / "dynamics_models" /
                               model_config.file);
      model_pipeline.emplace_back(std::make_shared<ModelLinCubDrag>(
        ModelLinCubDrag{quadrotor_, lin_cub_drag_params}));
    } else if (model_config.type == "ModelRigidBody") {
      model_pipeline.emplace_back(
        std::make_shared<ModelRigidBody>(ModelRigidBody{quadrotor_}));
    }
  }

  return true;
}

bool SimulatorParams::createLowLevelController(
  std::shared_ptr<LowLevelControllerBase>& ctrl) const {
  if (controller_name_.empty()) {
    // default low level controller
    LowLevelControllerSimpleParams params;
    ctrl =
      std::make_shared<LowLevelControllerSimple>(quadrotor_, sim_dt_, params);
    logger_.info("Enabled simple low level controller.");
    return true;
  } else {
    try {
      if (controller_name_ == "Simple") {
        LowLevelControllerSimpleParams params;
        if (!controller_file_.empty() &&
            !params.load(agi_param_directory_ / "low_level_controllers" /
                         controller_file_))
          throw ParameterException();
        ctrl = std::make_shared<LowLevelControllerSimple>(quadrotor_, sim_dt_,
                                                          params);
        logger_.info("Enabled simple low level controller.");
        return true;
      } else if (controller_name_ == "Betaflight") {
        LowLevelControllerBetaflightParams params;
        logger_.info("controller_file_: %s", controller_file_.string().c_str());
        if (controller_file_.empty() ||
            !params.load(agi_param_directory_ / "low_level_controllers" /
                         controller_file_) ||
            !params.loadThrustMap(agi_param_directory_))
          throw ParameterException();
        ctrl = std::make_shared<LowLevelControllerBetaflight>(quadrotor_,
                                                              sim_dt_, params);
        logger_.info("Enabled Betaflight low level controller.");
        return true;
      }
    } catch (const ParameterException& e) {
      throw ParameterException("Could not load low level controller " +
                               controller_name_ + " from parameter file \'" +
                               controller_file_.string() + "\':\n" + e.what());
    }
  }

  return false;
}

bool SimulatorParams::createIntegrator(
  std::shared_ptr<IntegratorBase>& integrator,
  DynamicsFunction dynamics_function) const {
  if (integrator_name_.empty()) {
    // default integrator
    LowLevelControllerSimpleParams params;
    integrator = std::make_shared<IntegratorRK4>(dynamics_function, sim_dt_);
    return true;
  } else {
    if (integrator_name_ == "Euler") {
      integrator =
        std::make_shared<IntegratorEuler>(dynamics_function, sim_dt_);
      logger_.info("Enabled Euler integrator with a step size of %.4f seconds.",
                   sim_dt_);
      return true;
    } else if (integrator_name_ == "RK4") {
      integrator = std::make_shared<IntegratorRK4>(dynamics_function, sim_dt_);
      logger_.info("Enabled RK4 integrator with a step size of %.4f seconds.",
                   sim_dt_);
      return true;
    } else if (integrator_name_ == "SymplecticEuler") {
      integrator =
        std::make_shared<IntegratorSymplecticEuler>(dynamics_function, sim_dt_);
      logger_.info(
        "Enabled SymplecticEuler integrator with a step size of %.4f seconds.",
        sim_dt_);
      return true;
    } else {
      logger_.error("Unknown integrator specified: [%s]",
                    integrator_name_.c_str());
      return false;
    }
  }
}

}  // namespace agi
