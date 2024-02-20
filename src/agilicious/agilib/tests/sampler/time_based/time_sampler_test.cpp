#include "agilib/sampler/time_based/time_sampler.hpp"

#include <gtest/gtest.h>

#include "agilib/reference/hover_reference.hpp"

using namespace agi;

TEST(TimeBasedSampler, Constructor) {
  int horizon_len = 10;
  Scalar horizon_dt = 0.1;
  TimeSampler sampler(horizon_len, horizon_dt);

  EXPECT_EQ(sampler.getHorizonLength(), horizon_len);
  EXPECT_EQ(sampler.getHorizonDt(), horizon_dt);
}

TEST(TimeBasedSampler, SampleSingleTrajectory) {
  // create a constant trajectory
  QuadState quad_state;
  quad_state.setZero();
  quad_state.t = 0.0;
  quad_state.p = Vector<3>(0.0, 0.0, 3.0);
  static constexpr Scalar duration = 2.0;
  HoverReference constant_trajectory = HoverReference(quad_state, duration);

  // generate a reference from the constant trajectory
  ReferenceVector references;
  references.push_back(std::make_shared<HoverReference>(constant_trajectory));
  static constexpr int horizon_len = 10;
  static constexpr Scalar horizon_dt = 0.1;
  TimeSampler sampler(horizon_len, horizon_dt);

  QuadState query_state;
  query_state.setZero();
  std::vector<Setpoint> setpoints;
  sampler.getAt(query_state, references, &setpoints);

  EXPECT_EQ(sampler.getHorizonLength(), setpoints.size());
  static constexpr int custom_horizon_len = 42;
  sampler.getAt(query_state, references, &setpoints, custom_horizon_len);
  EXPECT_EQ(custom_horizon_len, setpoints.size());

  EXPECT_TRUE(quad_state.x.isApprox(setpoints.front().state.x));
}

TEST(TimeBasedSampler, SampleMultipleTrajectories) {
  // create a constant trajectory
  static constexpr Scalar traj_z_1 = 3.0;
  static constexpr Scalar traj_z_2 = 5.0;
  QuadState quad_state;
  quad_state.setZero();
  quad_state.t = 0.0;
  quad_state.p = Vector<3>(0.0, 0.0, traj_z_1);
  static constexpr Scalar duration = 2.0;
  HoverReference constant_trajectory_1 = HoverReference(quad_state, duration);

  QuadState quad_state_2;
  quad_state_2.setZero();
  quad_state_2.t = constant_trajectory_1.getEndTime();
  quad_state_2.p = Vector<3>(0.0, 0.0, traj_z_2);
  HoverReference constant_trajectory_2 = HoverReference(quad_state_2, duration);

  // generate a reference from the constant trajectory
  ReferenceVector references;
  references.push_back(std::make_shared<HoverReference>(constant_trajectory_1));
  references.push_back(std::make_shared<HoverReference>(constant_trajectory_2));
  static constexpr int horizon_len = 10;
  static constexpr Scalar horizon_dt = 0.1;
  TimeSampler sampler(horizon_len, horizon_dt);

  QuadState query_state;
  query_state.setZero();
  std::vector<Setpoint> setpoints;

  // sample first part of trajectory
  sampler.getAt(query_state, references, &setpoints);
  EXPECT_EQ(sampler.getHorizonLength(), setpoints.size());
  EXPECT_TRUE(quad_state.x.isApprox(setpoints.front().state.x));

  // sample over trajectory intersection
  query_state.t = 1.5;
  sampler.getAt(query_state, references, &setpoints);
  EXPECT_EQ(sampler.getHorizonLength(), setpoints.size());
  EXPECT_TRUE(quad_state.x.isApprox(setpoints.front().state.x));
  EXPECT_TRUE(quad_state_2.x.isApprox(setpoints.back().state.x));

  // sample over end of trajectory
  static constexpr int custom_horizon_len = 42;
  query_state.t = 2.5;
  sampler.getAt(query_state, references, &setpoints, custom_horizon_len);
  EXPECT_EQ(custom_horizon_len, setpoints.size());
  EXPECT_TRUE(quad_state_2.x.isApprox(setpoints.front().state.x));
  EXPECT_TRUE(quad_state_2.x.isApprox(setpoints.back().state.x));

  // sample over entire reference
  Scalar t_curr = 0.0;
  while (t_curr < 3.0) {
    query_state.t = t_curr;
    EXPECT_TRUE(sampler.getAt(query_state, references, &setpoints));
    t_curr += 0.02;
  }
}
