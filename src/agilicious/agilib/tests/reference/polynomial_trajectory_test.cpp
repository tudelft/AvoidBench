
#include "agilib/reference/trajectory_reference/polynomial_trajectory.hpp"

#include <gtest/gtest.h>

using namespace agi;


TEST(PolynomialTrajectory, BodyrateTestHeading0) {
  static constexpr Scalar offset = 5.0;
  static constexpr Scalar T = 1.5;
  static constexpr Scalar t0 = 1.5e6;
  static constexpr Scalar t1 = t0 + T;
  static constexpr Scalar tm = t0 + 0.5 * T;
  static constexpr Scalar dt = 0.1;
  static constexpr Scalar heading = 0.0;

  const QuadState start(t0, Vector<3>::Zero(), heading);
  const QuadState state_positive_x(t1, Vector<3>(offset, 0, 0), heading);
  const QuadState state_positive_y(t1, Vector<3>(0, offset, 0), heading);
  const QuadState state_negative_x(t1, Vector<3>(-offset, 0, 0), heading);
  const QuadState state_negative_y(t1, Vector<3>(0, -offset, 0), heading);

  PolynomialTrajectory<> traj_positive_x(start, state_positive_x);
  PolynomialTrajectory<> traj_positive_y(start, state_positive_y);
  PolynomialTrajectory<> traj_negative_x(start, state_negative_x);
  PolynomialTrajectory<> traj_negative_y(start, state_negative_y);

  EXPECT_TRUE(traj_positive_x.solved());
  EXPECT_TRUE(traj_positive_y.solved());
  EXPECT_TRUE(traj_negative_x.solved());
  EXPECT_TRUE(traj_negative_y.solved());

  EXPECT_GT(traj_positive_x.getState(t0 + dt).w.y(), 0.0);
  EXPECT_NEAR(traj_positive_x.getState(t0 + dt).w.x(), 0.0, 1e-3);
  EXPECT_LT(traj_positive_y.getState(t0 + dt).w.x(), 0.0);
  EXPECT_NEAR(traj_positive_y.getState(t0 + dt).w.y(), 0.0, 1e-3);
  EXPECT_LT(traj_negative_x.getState(t0 + dt).w.y(), 0.0);
  EXPECT_NEAR(traj_negative_x.getState(t0 + dt).w.x(), 0.0, 1e-3);
  EXPECT_GT(traj_negative_y.getState(t0 + dt).w.x(), 0.0);
  EXPECT_NEAR(traj_negative_y.getState(t0 + dt).w.y(), 0.0, 1e-3);

  EXPECT_LT(traj_positive_x.getState(tm).w.y(), 0.0);
  EXPECT_GT(traj_positive_y.getState(tm).w.x(), 0.0);
  EXPECT_GT(traj_negative_x.getState(tm).w.y(), 0.0);
  EXPECT_LT(traj_negative_y.getState(tm).w.x(), 0.0);
}

TEST(PolynomialTrajectory, BodyrateTestHeading90) {
  static constexpr Scalar offset = 5.0;
  static constexpr Scalar T = 1.5;
  static constexpr Scalar t0 = 1.5e6;
  static constexpr Scalar t1 = t0 + T;
  static constexpr Scalar tm = t0 + 0.5 * T;
  static constexpr Scalar dt = 0.1;
  static constexpr Scalar heading = M_PI / 2.0;

  const QuadState start(t0, Vector<3>::Zero(), heading);
  const QuadState state_positive_x(t1, Vector<3>(offset, 0, 0), heading);
  const QuadState state_positive_y(t1, Vector<3>(0, offset, 0), heading);
  const QuadState state_negative_x(t1, Vector<3>(-offset, 0, 0), heading);
  const QuadState state_negative_y(t1, Vector<3>(0, -offset, 0), heading);

  PolynomialTrajectory<> traj_positive_x(start, state_positive_x);
  PolynomialTrajectory<> traj_positive_y(start, state_positive_y);
  PolynomialTrajectory<> traj_negative_x(start, state_negative_x);
  PolynomialTrajectory<> traj_negative_y(start, state_negative_y);

  EXPECT_TRUE(traj_positive_x.solved());
  EXPECT_TRUE(traj_positive_y.solved());
  EXPECT_TRUE(traj_negative_x.solved());
  EXPECT_TRUE(traj_negative_y.solved());

  EXPECT_GT(traj_positive_x.getState(t0 + dt).w.x(), 0.0);
  EXPECT_NEAR(traj_positive_x.getState(t0 + dt).w.y(), 0.0, 1e-3);
  EXPECT_GT(traj_positive_y.getState(t0 + dt).w.y(), 0.0);
  EXPECT_NEAR(traj_positive_y.getState(t0 + dt).w.x(), 0.0, 1e-3);
  EXPECT_LT(traj_negative_x.getState(t0 + dt).w.x(), 0.0);
  EXPECT_NEAR(traj_negative_x.getState(t0 + dt).w.y(), 0.0, 1e-3);
  EXPECT_LT(traj_negative_y.getState(t0 + dt).w.y(), 0.0);
  EXPECT_NEAR(traj_negative_y.getState(t0 + dt).w.x(), 0.0, 1e-3);

  EXPECT_LT(traj_positive_x.getState(tm).w.x(), 0.0);
  EXPECT_LT(traj_positive_y.getState(tm).w.y(), 0.0);
  EXPECT_GT(traj_negative_x.getState(tm).w.x(), 0.0);
  EXPECT_GT(traj_negative_y.getState(tm).w.y(), 0.0);
}
