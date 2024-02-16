#pragma once

#include "agilib/base/pipeline.hpp"
#include "agilib/bridge/debug_bridge.hpp"
#include "agilib/guard/position_guard.hpp"
#include "agilib/pilot/pilot_params.hpp"
#include "agilib/reference/velocity_reference.hpp"
#include "agilib/types/quadrotor.hpp"
#include "agilib/utils/logger.hpp"
#include "agilib/utils/timer.hpp"
#include "agilib/reference/nw_poly_trajectory_reference.hpp"

namespace agi {

class Pilot {
 public:
  Pilot(const PilotParams& params, const TimeFunction time_function);
  ~Pilot();

  void launchPipeline();
  void runPipeline();
  void runPipeline(const Scalar t);


  bool getReference(const int idx, ReferenceBase* const reference) const;
  bool getActiveReference(const QuadState& curr_state,
                          SetpointVector* const setpoints) const;
  int getAllReferences(ReferenceVector* const references) const;

  bool registerExternalEstimator(
    const std::shared_ptr<EstimatorBase>& estimator);
  bool registerExternalBridge(const std::shared_ptr<BridgeBase>& bridge);
  bool registerFeedbackCallbacks(
    const std::shared_ptr<BridgeBase>& bridge) const;
  bool registerFeedbackCallback(const FeedbackCallbackFunction& function);
  bool registerExternalDebugBridge(const std::shared_ptr<BridgeBase>& bridge);
  void registerPipelineCallback(
    const Pipeline::PipelineCallbackFunction& function);

  void odometryCallback(const Pose& pose);
  void odometryCallback(const QuadState& state);
  void guardOdometryCallback(const Pose& pose);
  void guardOdometryCallback(const QuadState& state);
  void voltageCallback(const Scalar voltage);
  void imuCallback(const ImuSample& imu);
  void motorSpeedCallback(const Vector<4>& mot);
  void ctrActivateCallback(const bool activate);

  Command getCommand() const;
  const SetpointVector getOuterSetpoints() const;
  const SetpointVector getInnerSetpoints() const;
  QuadState getRecentState() const;
  Scalar getVoltage() const;

  void enable(bool enable);
  bool enabled();

  bool start();
  bool land();
  bool off();


  Quaternion EularToquaternion(const Vector<3> euler);
  bool forceHover();
  bool goToPose(const QuadState& end_state);
  bool setVelocityReference(const Vector<3>& velocity, const Scalar yaw_rate);
  bool setAccelerationCommand(const Vector<3>& acceleration,
                                   const Scalar yaw_rate, const Scalar delta_t, const Scalar new_time);
  bool setAccelerationCommand(const std::vector<Vector<3>>& acceleration,
                                    const std::vector<Scalar>& yaw_rate, const Scalar delta_t, const Scalar new_time);
  bool setRLtrajectoryCommand(const SetpointVector& sampled_trajectory, const QuadState& inference,
                              const Scalar time_received_prediction, const std::string frame_type);

  bool addHover(const Vector<3>& hover_pos, Scalar yaw = NAN,
                Scalar start_time = NAN, Scalar duration = NAN);

  template<typename ReferenceType>
  bool addReference(const ReferenceType& reference) {
    return pipeline_.appendReference(reference);
  }
  bool appendTrajectory(const QuadState& start_state,
                        const QuadState& end_state);
  bool addPolynomialTrajectory(const QuadState& end_state,
                               const Scalar duration);

  bool addPolynomialTrajectory(const QuadState& end_state,
                              const Scalar duration, const Scalar new_time);

  bool addPolynomialTrajectory(const QuadState& start, const QuadState& end,
                               const std::vector<QuadState>& end_state,
                               const Scalar speed,
                               const Scalar limit_scale = 0.0);

  bool addPolynomialTrajectory(const QuadState& start, const QuadState& end,
                               const std::vector<QuadState>& end_state,
                               const Vector<> t,
                               const Scalar limit_scale = 0.0);

  bool addSampledTrajectory(const SetpointVector& setpoints);

  bool setFeedthroughCommand(const Command& command);

  void convertTrajectoryToWorldFrame(const SetpointVector& trajectory, const QuadState& odom_at_inference,
    SetpointVector* trajectory_world_frame, const std::string frame_type);

  bool isInHover() const;
  bool isInVelocityReference() const;
  std::string getActiveBridgeType();

  bool getFeedback(Feedback* const feedback = nullptr) const;

  bool getQuadrotor(Quadrotor* const quad) const;
  bool guardTriggered() { return guard_ && guard_->triggered(); }
  void triggerGuard();
  inline const PilotParams& getParams() const { return params_; }
  inline Scalar getTime() const { return time_(); }

 private:
  void pipelineThread();
  Pipeline& getActivePipeline();
  const TimeFunction time_;
  PilotParams params_;
  Pipeline pipeline_;
  Pipeline safety_pipeline_;
  std::shared_ptr<GuardBase> guard_;
  std::shared_ptr<BridgeBase> bridge_;
  std::shared_ptr<BridgeBase> debug_bridge_;

  bool shutdown_{false};
  std::thread pipeline_thread_;
  Timer pipeline_timer_;
  QuadState curr_reference_;
  bool reference_ready_{false};
  TrajectoryExt network_prediction_;
  Scalar time_received_prediction_;
  Logger logger_{"Pilot"};
};


}  // namespace agi
