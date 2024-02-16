#include "agilib/base/pipeline.hpp"

#include <gtest/gtest.h>

#include "agilib/reference/hover_reference.hpp"

using namespace agi;

class DumbReference : public ReferenceBase {
  Setpoint getSetpoint(const QuadState &state, const Scalar t) {
    return Setpoint();
  }
  bool valid() const { return true; };
};

class DumbEstimator : public EstimatorBase {
  bool getAt(const Scalar t, QuadState *const) { return true; }
  bool initialize(const QuadState &) { return true; }
  bool addPose(const Pose &) { return true; }
  bool addImu(const ImuSample &) { return true; }
  bool addMotorSpeeds(const Vector<4> &) { return true; }
  bool addState(const QuadState &) { return true; }
  bool healthy() const { return true; }
};

class DumbSampler : public SamplerBase {
  bool getAt(const QuadState &state, const ReferenceVector &references,
             SetpointVector *const setpoint, int horizon_len = -1) const {
    return true;
  }

  bool isTimeBased() const { return true; }
};

class DumbController : public ControllerBase {
  bool getCommand(const QuadState &state, const SetpointVector &references,
                  SetpointVector *const setpoints) {
    return true;
  }
};

class DumbBridge : public BridgeBase {
 public:
  DumbBridge(const std::string &name, const TimeFunction time_function)
    : BridgeBase(name, time_function){};

 private:
  bool performActivate() { return true; }
  bool performDeactivate() { return true; }
  bool sendCommand(const Command &command, const bool active = false) override {
    return true;
  }
};

TEST(Pipeline, Constructor) {
  Pipeline pipeline;

  EXPECT_FALSE(pipeline.isReferenceSet());
  EXPECT_FALSE(pipeline.isEstimatorSet());
  EXPECT_FALSE(pipeline.isSamplerSet());
  EXPECT_FALSE(pipeline.isOuterControllerSet());
  EXPECT_FALSE(pipeline.isInnerControllerSet());
  EXPECT_FALSE(pipeline.isBridgeSet());
  EXPECT_FALSE(pipeline.isSet());

  EXPECT_FALSE(pipeline.run(0.0));
}

TEST(Pipeline, Setup) {
  Pipeline pipeline;
  pipeline.references_ = ReferenceVector{std::make_shared<DumbReference>()};
  pipeline.estimator_ = std::make_shared<DumbEstimator>();
  pipeline.sampler_ = std::make_shared<DumbSampler>();
  pipeline.outer_controller_ = std::make_shared<DumbController>();
  pipeline.bridge_ = std::make_shared<DumbBridge>("DumbBridge", ChronoTime);

  EXPECT_TRUE(pipeline.isReferenceSet());
  EXPECT_TRUE(pipeline.isEstimatorSet());
  EXPECT_TRUE(pipeline.isSamplerSet());
  EXPECT_TRUE(pipeline.isOuterControllerSet());
  EXPECT_FALSE(pipeline.isInnerControllerSet());
  EXPECT_TRUE(pipeline.isBridgeSet());
  EXPECT_TRUE(pipeline.isSet());

  EXPECT_TRUE(pipeline.run(0.0));
}
