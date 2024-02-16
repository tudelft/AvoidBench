#pragma once

#include <memory>
#include <string>

#include "agilib/base/parameter_base.hpp"
#include "agilib/base/pipeline.hpp"
#include "agilib/bridge/bridge_base.hpp"
#include "agilib/controller/controller_base.hpp"
#include "agilib/estimator/estimator_base.hpp"
#include "agilib/guard/guard_base.hpp"
#include "agilib/pilot/pipeline_config.hpp"
#include "agilib/sampler/sampler_base.hpp"
#include "agilib/types/quadrotor.hpp"

namespace agi {

class PilotParams : public ParameterBase {
 public:
  PilotParams() = default;
  PilotParams(const fs::path& filename, const fs::path& directory = "",
              const fs::path& quad_file = "");

  using ParameterBase::load;
  bool load(const Yaml& node) override;

  bool valid() const override;

  bool createEstimator(std::shared_ptr<EstimatorBase>& estimator,
                       const ModuleConfig& config) const;
  bool createSampler(std::shared_ptr<SamplerBase>& sampler,
                     const std::shared_ptr<ControllerBase>& controller,
                     const ModuleConfig& config) const;
  bool createController(std::shared_ptr<ControllerBase>& controller,
                        const ModuleConfig& config) const;
  bool createBridge(std::shared_ptr<BridgeBase>& bridge,
                    const TimeFunction& time_function,
                    const ModuleConfig&) const;
  bool createBridge(std::shared_ptr<BridgeBase>& bridge,
                    const TimeFunction& time_function) const;

  bool createPipeline(Pipeline* const pipeline,
                      const PipelineConfig& config) const;
  bool createPipeline(Pipeline* const pipeline) const;
  bool createSafetyPipeline(Pipeline* const safety_pipeline) const;

  bool createGuard(std::shared_ptr<GuardBase>& guard) const;

  fs::path directory_;

  PipelineConfig pipeline_cfg_;
  PipelineConfig safety_pipeline_cfg_;

  ModuleConfig guard_cfg_;

  std::string traj_type_{"poly_min_snap"};
  fs::path quad_file_;

  Quadrotor quad_;

  Scalar dt_min_{0.01};
  int outerloop_divisor_{1};
  Scalar dt_telemetry_{0.1};

  bool velocity_in_bodyframe_{true};
  Scalar takeoff_heigth_{1.0};
  Scalar takeoff_threshold_{0.5};
  Scalar start_land_speed_{0.6};
  Scalar brake_deceleration_{5.0};
  Scalar go_to_pose_mean_vel_{1.5};
  bool stop_after_feedthrough_{true};
  Scalar feedthrough_timeout_{0.05};

  Scalar traj_viz_dt_{0.1};
  Scalar traj_viz_sphere_size_{0.1};

  bool publish_log_var_{false};

  friend std::ostream& operator<<(std::ostream& os, const PilotParams& params);

 private:
  Logger logger_{"PilotParams"};
};


}  // namespace agi
