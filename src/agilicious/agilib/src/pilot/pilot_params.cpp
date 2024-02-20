#include "agilib/pilot/pilot_params.hpp"

#include <unistd.h>

#include <filesystem>
#include <memory>

#include "agilib/base/parameter_base.hpp"
#include "agilib/bridge/ctrl/ctrl_bridge.hpp"
#include "agilib/bridge/ctrl/ctrl_bridge_params.hpp"
#include "agilib/bridge/debug_bridge.hpp"
#include "agilib/bridge/laird/laird_bridge.hpp"
#include "agilib/bridge/laird/laird_bridge_params.hpp"
#include "agilib/bridge/msp/msp_bridge.hpp"
#include "agilib/bridge/msp/msp_bridge_params.hpp"
#include "agilib/bridge/sbus/sbus_bridge.hpp"
#include "agilib/bridge/sbus/sbus_bridge_params.hpp"
#include "agilib/controller/geometric/controller_geo.hpp"
#include "agilib/controller/geometric/geo_params.hpp"
#include "agilib/controller/indi/controller_indi.hpp"
#include "agilib/controller/indi/indi_params.hpp"
#include "agilib/controller/mpc/controller_mpc.hpp"
#include "agilib/controller/mpc/mpc_params.hpp"
#include "agilib/controller/pid/controller_pid.hpp"
#include "agilib/controller/pid/pid_params.hpp"
#include "agilib/estimator/ekf/ekf.hpp"
#include "agilib/estimator/ekf/ekf_params.hpp"
#include "agilib/estimator/ekf_imu/ekf_imu.hpp"
#include "agilib/estimator/ekf_imu/ekf_imu_params.hpp"
#include "agilib/estimator/feedthrough/feedthrough_estimator.hpp"
#include "agilib/estimator/feedthrough/feedthrough_params.hpp"
#include "agilib/estimator/mock_vio/mock_vio.hpp"
#include "agilib/estimator/mock_vio/mock_vio_params.hpp"
#include "agilib/guard/position_guard.hpp"
#include "agilib/guard/position_guard_params.hpp"
#include "agilib/pilot/pipeline_config.hpp"
#include "agilib/sampler/position_based/position_sampler.hpp"
#include "agilib/sampler/position_based/position_sampler_params.hpp"
#include "agilib/sampler/time_based/time_sampler.hpp"
#include "agilib/utils/file_utils.hpp"

namespace agi {

PilotParams::PilotParams(const fs::path& filename, const fs::path& directory,
                         const fs::path& quad_file)
  : directory_(directory), quad_file_(quad_file) {
  fs::path full_filename = filename;
  if (!checkFile(directory_, &full_filename))
    throw ParameterException("Pilot Config file not found!\n" +
                             filename.string() + "\n" + full_filename.string());

  logger_.info("Loading Pilot parameters from [%s].", full_filename.c_str());
  if (!ParameterBase::load(full_filename))
    throw ParameterException(full_filename);
}

bool PilotParams::load(const Yaml& yaml) {
  if (directory_.empty()) {
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
          directory_ /= *it;
        directory_ = directory_ / "agilicious" / "agilib" / "params";
      }
    }
  }
  pipeline_cfg_.load(yaml["pipeline"], directory_);

  // Load Guard Parameters
  if (guard_cfg_.loadIfUndefined(yaml["guard"])) {
    if (!guard_cfg_.file.empty() && !checkFile(directory_, &guard_cfg_.file))
      throw ParameterException("Could not find guard config file!");
    safety_pipeline_cfg_.load(yaml["guard"]["pipeline"], directory_);
  }

  // Quadrotor
  quad_file_ = getQuadFile(yaml, quad_file_, directory_);
  if (!quad_.load(quad_file_) || !quad_.valid())
    throw ParameterException("Could not load Quadrotor parameters from: " +
                             quad_file_.string());

  // Pilot Params
  dt_min_ = yaml["dt_min"].as<Scalar>();
  yaml["outerloop_divisor"].getIfDefined(outerloop_divisor_);
  dt_telemetry_ = yaml["dt_telemetry"].as<Scalar>();
  traj_type_ = yaml["traj_type"].as<std::string>();

  yaml["velocity_in_bodyframe"].getIfDefined(velocity_in_bodyframe_);
  yaml["takeoff_height"].getIfDefined(takeoff_heigth_);
  yaml["takeoff_threshold"].getIfDefined(takeoff_threshold_);
  yaml["start_land_speed"].getIfDefined(start_land_speed_);
  yaml["brake_deceleration"].getIfDefined(brake_deceleration_);
  yaml["go_to_pose_mean_vel"].getIfDefined(go_to_pose_mean_vel_);
  yaml["stop_after_feedthrough"].getIfDefined(stop_after_feedthrough_);
  yaml["feedthrough_timeout"].getIfDefined(feedthrough_timeout_);

  yaml["viz_sampler_dt"].getIfDefined(traj_viz_dt_);
  yaml["sphere_size"].getIfDefined(traj_viz_sphere_size_);
  yaml["pub_log_var"].getIfDefined(publish_log_var_);

  return valid();
}

bool PilotParams::valid() const { return quad_.valid(); }

bool PilotParams::createEstimator(std::shared_ptr<EstimatorBase>& estimator,
                                  const ModuleConfig& config) const {
  try {
    if (config.type == "EKF") {
      std::shared_ptr<EkfParameters> params = std::make_shared<EkfParameters>();
      if (!config.file.empty() && !params->load(config.file))
        throw ParameterException();
      estimator = std::make_shared<Ekf>(quad_, params);
      return true;
    } else if (config.type == "EKFIMU") {
      std::shared_ptr<EkfImuParameters> params =
        std::make_shared<EkfImuParameters>();
      if (!config.file.empty() && !params->load(config.file))
        throw ParameterException();
      estimator = std::make_shared<EkfImu>(params);
      return true;
    } else if (config.type == "Feedthrough") {
      std::shared_ptr<FeedthroughParameters> params =
        std::make_shared<FeedthroughParameters>();
      if (!config.file.empty() && !params->load(config.file))
        throw ParameterException();
      estimator = std::make_shared<FeedthroughEstimator>(params);
      return true;
    } else if (config.type == "MockVIO") {
      std::shared_ptr<MockVioParams> params = std::make_shared<MockVioParams>();
      if (!config.file.empty() && !params->load(config.file))
        throw ParameterException();
      estimator = std::make_shared<MockVio>(quad_, params);
      return true;
    }
  } catch (const ParameterException& e) {
    throw ParameterException("Could not load estimator " + config.type +
                             " from parameter file \'" + config.file.string() +
                             "\':\n" + e.what());
  }

  return false;
}

bool PilotParams::createController(std::shared_ptr<ControllerBase>& controller,
                                   const ModuleConfig& config) const {
  try {
    if (config.type == "MPC") {
      std::shared_ptr<MpcParameters> params = std::make_shared<MpcParameters>();
      if (!config.file.empty() && !params->load(config.file))
        throw ParameterException();
      std::cout<<"dt_min_: "<<dt_min_<<std::endl;
      controller = std::make_shared<MpcController>(quad_, params, dt_min_);
      return true;
    } else if (config.type == "INDI") {
      std::shared_ptr<IndiParameters> params =
        std::make_shared<IndiParameters>();
      if (!config.file.empty() && !params->load(config.file))
        throw ParameterException();
      controller = std::make_shared<IndiController>(quad_, params);
      return true;
    } else if (config.type == "GEO") {
      std::shared_ptr<GeometricControllerParams> params =
        std::make_shared<GeometricControllerParams>();
      if (!config.file.empty() && !params->load(config.file))
        throw ParameterException();
      controller = std::make_shared<GeometricController>(quad_, params);
      return true;
    } else if (config.type == "PID") {
      std::shared_ptr<PidParameters> params = std::make_shared<PidParameters>();
      if (!config.file.empty() && !params->load(config.file))
        throw ParameterException();
      controller = std::make_shared<PidController>(quad_, params);
      return true;
    }
  } catch (const ParameterException& e) {
    throw ParameterException("Could not load controller " + config.type +
                             " from parameter file \'" + config.file.string() +
                             "\':\n" + e.what());
  }

  return false;
}

bool PilotParams::createBridge(std::shared_ptr<BridgeBase>& bridge,
                               const TimeFunction& time_function,
                               const ModuleConfig& config) const {
  try {
    if (config.type == "SBUS") {
      SbusParams params;
      if (!config.file.empty() && !params.load(config.file))
        throw ParameterException();
      bridge = std::make_shared<SbusBridge>(quad_, params, time_function);
      return true;
    } else if (config.type == "Laird") {
      LairdParams params;
      if (!config.file.empty() && !params.load(config.file))
        throw ParameterException();
      bridge = std::make_shared<LairdBridge>(params, time_function);
      return true;
    } else if (config.type == "Debug") {
      bridge = std::make_shared<DebugBridge>("DebugBridge", time_function);
      return true;
    } else if (config.type == "MSP") {
      MspParams params;
      if (!config.file.empty() && !params.load(config.file))
        throw ParameterException();
      bridge = std::make_shared<MspBridge>(quad_, params, time_function);
      return true;
    } else if (config.type == "CTRL") {
      CtrlParams params;
      if (!config.file.empty() && !params.load(config.file))
        throw ParameterException();
      bridge = std::make_shared<CtrlBridge>(quad_, params, time_function);
      return true;
    }
  } catch (const ParameterException& e) {
    throw ParameterException("Could not load bridge " + config.type +
                             " from parameter file \'" + config.file.string() +
                             "\':\n" + e.what());
  }

  return false;
}

bool PilotParams::createBridge(std::shared_ptr<BridgeBase>& bridge,
                               const TimeFunction& time_function) const {
  return createBridge(bridge, time_function, pipeline_cfg_.bridge_cfg);
}

bool PilotParams::createSampler(
  std::shared_ptr<SamplerBase>& sampler,
  const std::shared_ptr<ControllerBase>& controller,
  const ModuleConfig& config) const {
  if (!controller) return false;
  try {
    if (config.type == "Time") {
      std::cout<<"controller->dt(): "<<controller->dt()<<std::endl;
      sampler = std::make_shared<TimeSampler>(controller->horizonLength(),
                                              controller->dt());
      return true;
    } else if (config.type == "Position") {
      PositionSamplerParameters params;
      if (!config.file.empty() && !params.load(config.file))
        throw ParameterException();
      sampler = std::make_shared<PositionSampler>(
        params, controller->horizonLength(), controller->dt());
      return true;
    }
  } catch (const ParameterException& e) {
    throw ParameterException("Could not load sampler " + config.type +
                             " from parameter file \'" + config.file.string() +
                             "\':\n" + e.what());
  }

  return false;
}

bool PilotParams::createPipeline(Pipeline* const pipeline,
                                 const PipelineConfig& config) const {
  if (pipeline == nullptr) return false;
  pipeline->setOuterloopDivisor(outerloop_divisor_);
  pipeline->setStopAfterFeedthrough(stop_after_feedthrough_);
  pipeline->setFeedthroughTimeout(feedthrough_timeout_);

  if (!createEstimator(pipeline->estimator_, pipeline_cfg_.estimator_cfg))
    logger_.warn("Did not create estimator '%s'!",
                 pipeline_cfg_.estimator_cfg.type.c_str());
  if (!createController(pipeline->outer_controller_,
                        pipeline_cfg_.outer_controller_cfg))
    logger_.warn("Did not create outer controller '%s'!",
                 pipeline_cfg_.outer_controller_cfg.type.c_str());
  if (!createController(pipeline->inner_controller_,
                        pipeline_cfg_.inner_controller_cfg))
    logger_.warn("Did not create inner controller '%s'!",
                 pipeline_cfg_.inner_controller_cfg.type.c_str());
  if (!createSampler(pipeline->sampler_, pipeline->outer_controller_,
                     pipeline_cfg_.sampler_cfg))
    logger_.warn("Did not create sampler '%s'!",
                 pipeline_cfg_.sampler_cfg.type.c_str());

  return true;
}

bool PilotParams::createPipeline(Pipeline* const pipeline) const {
  return createPipeline(pipeline, pipeline_cfg_);
}

bool PilotParams::createSafetyPipeline(Pipeline* safety_pipeline) const {
  return createPipeline(safety_pipeline, safety_pipeline_cfg_);
}

bool PilotParams::createGuard(std::shared_ptr<GuardBase>& guard) const {
  try {
    if (guard_cfg_.type == "Position") {
      PositionGuardParams params;
      if (!guard_cfg_.file.empty() && !params.load(guard_cfg_.file))
        throw ParameterException();
      guard = std::make_shared<PositionGuard>(params);
      return true;
    } else if (guard_cfg_.type == "None") {
      return true;
    }
  } catch (const ParameterException& e) {
    throw ParameterException("Could not load guard " + guard_cfg_.type +
                             " from parameter file \'" +
                             guard_cfg_.file.string() + "\':\n" + e.what());
  }

  return false;
}

std::ostream& operator<<(std::ostream& os, const PilotParams& params) {
  os << "Pilot Parameters:\n";
  os << "Directory:                    " << params.directory_ << '\n';
  os << '\n';
  os << "Quad File:                    " << params.quad_file_ << '\n';
  os << '\n';
  os << "Pipeline:\n" << params.pipeline_cfg_;
  os << '\n';
  os << "Safety Pipeline:\n" << params.safety_pipeline_cfg_;
  os << '\n';
  os << "Guard\n" << params.guard_cfg_;
  os << '\n';

  os << "Trajectory Type:              " << params.traj_type_ << '\n';
  os << "dt min:                       " << params.dt_min_ << '\n';
  os << "dt telemetry:                 " << params.dt_telemetry_ << '\n';
  os << "outerloop divisor:            " << params.outerloop_divisor_ << '\n';
  os << "velocity in bodyframe:        " << params.velocity_in_bodyframe_
     << '\n';
  os << "takeoff height:               " << params.takeoff_heigth_ << '\n';
  os << "takeoff threshold:            " << params.takeoff_threshold_ << '\n';
  os << "start land speed:             " << params.start_land_speed_ << '\n';
  os << "brake deceleration:           " << params.brake_deceleration_ << '\n';
  os << "go to pose velocity:          " << params.go_to_pose_mean_vel_ << '\n';
  os << "stop after feedthough:        " << params.stop_after_feedthrough_
     << '\n';
  os << "trajectory visualization dt:  " << params.traj_viz_dt_ << '\n';
  os << "trajectory sphere size:       " << params.traj_viz_sphere_size_
     << '\n';
  os << "publish log variables:        " << params.publish_log_var_ << '\n';


  return os;
}

}  // namespace agi
