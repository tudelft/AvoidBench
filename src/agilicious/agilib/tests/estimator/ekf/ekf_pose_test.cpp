#include <gtest/gtest.h>

#include <random>

#include "agilib/estimator/ekf/ekf.hpp"
#include "agilib/math/gravity.hpp"

using namespace agi;

static constexpr Scalar M = 1.0;
static constexpr Scalar L = 0.25;
static constexpr int N = 320;
static constexpr Scalar DT = 0.01;
static constexpr Scalar WX = 0.5;
static constexpr Scalar WZ = 0.7;
static constexpr Scalar VX = 0.5;

TEST(EkfPose, Constructor) {
  std::shared_ptr<EkfParameters> params = std::make_shared<EkfParameters>();
  Quadrotor quad(M, L);
  Ekf ekf(quad, params);

  QuadState state;
  state.t = 0.0;
  EXPECT_FALSE(ekf.getRecent(&state));
  EXPECT_FALSE(ekf.getState(&state));
  EXPECT_FALSE(ekf.getAt(state.t, &state));

  EXPECT_FALSE(ekf.initialize(state));
  EXPECT_FALSE(ekf.healthy());
}

TEST(EkfPose, Initializing) {
  std::shared_ptr<EkfParameters> params = std::make_shared<EkfParameters>();
  Quadrotor quad(M, L);
  Ekf ekf(quad, params);

  EXPECT_FALSE(ekf.healthy());
  QuadState state;
  EXPECT_FALSE(ekf.initialize(state));
  state.setZero();
  EXPECT_TRUE(ekf.initialize(state));
  EXPECT_TRUE(ekf.healthy());
  EXPECT_TRUE(ekf.getRecent(&state));
  EXPECT_TRUE(state.valid());

  QuadState expected;
  expected.setZero();
  EXPECT_EQ(state, expected);
}

TEST(EkfPose, AddingPose) {
  std::shared_ptr<EkfParameters> params = std::make_shared<EkfParameters>();
  Quadrotor quad(M, L);
  Ekf ekf(quad, params);

  EXPECT_FALSE(ekf.healthy());
  const Pose init_pose{0.0, Vector<3>::Zero(), Quaternion(1, 0, 0, 0)};
  EXPECT_TRUE(ekf.addPose(init_pose));
  EXPECT_TRUE(ekf.healthy());

  Pose pose{0.0, Vector<3>::Zero(), Quaternion(1, 0, 0, 0)};
  for (int i = 0; i < N; ++i) {
    pose.t += DT;
    EXPECT_TRUE(ekf.addPose(pose));
  }

  pose.t = 0.0;
  EXPECT_FALSE(ekf.addPose(pose));
}

/// Move in positive x for 32 steps of 0.1s at 0.5 m/s.
TEST(EkfPose, UpdateTestPosition) {
  std::shared_ptr<EkfParameters> params = std::make_shared<EkfParameters>();
  Quadrotor quad(M, L);
  Ekf ekf(quad, params);
  QuadState state;
  state.setZero();
  state.v.x() = VX;

  EXPECT_FALSE(ekf.healthy());
  EXPECT_TRUE(ekf.initialize(state));
  EXPECT_TRUE(ekf.healthy());

  Pose pose{0.0, Vector<3>::Zero(), Quaternion(1, 0, 0, 0)};
  for (int i = 1; i <= N; ++i) {
    for (int j = 1; j <= 4; j++) {
      static constexpr Scalar DT4 = DT / 4.0;
      state.t += DT4;
      state.p += DT4 * state.v;
      state.linearize();
      const ImuSample sample{state.t, -GVEC, Vector<3>::Zero()};
      EXPECT_TRUE(ekf.addImu(sample));
    }

    pose.t = state.t;
    pose.attitude = state.q();
    pose.position = state.p;
    EXPECT_TRUE(ekf.addPose(pose));
    EXPECT_TRUE(ekf.healthy());
    const QuadState latest = ekf.getRecent();
    EXPECT_TRUE((latest.p - pose.position).isZero(1e-3))
      << "At time:   " << pose.t << std::endl
      << "Actual:    " << latest.p.transpose() << std::endl
      << "Expected:  " << pose.position.transpose() << std::endl
      << latest << std::endl;
    EXPECT_TRUE((latest.q().coeffs() - pose.attitude.coeffs()).isZero(1e-3))
      << "At time:   " << pose.t << std::endl
      << "Actual:    " << latest.q().coeffs().transpose() << std::endl
      << "Expected:  " << pose.attitude.coeffs().transpose() << std::endl
      << latest << std::endl;
  }

  QuadState estimated;
  EXPECT_TRUE(ekf.getRecent(&estimated));
  QuadState expected;
  expected.setZero();
  expected.t = (Scalar)N * DT;
  expected.p.x() = (Scalar)N * DT * VX;
  expected.v.x() = VX;

  EXPECT_TRUE(estimated.isApprox(expected, 1e-6));
}

// Rotating along positive x over 32 steps of 0.1s with 0.5rad/s.
TEST(EkfPose, UpdateTestRotation) {
  std::shared_ptr<EkfParameters> params = std::make_shared<EkfParameters>();
  Quadrotor quad(M, L);
  Ekf ekf(quad, params);
  QuadState state;
  state.setZero();
  state.w.x() = WX;
  state.w.z() = WZ;

  EXPECT_FALSE(ekf.healthy());
  EXPECT_TRUE(ekf.initialize(state));
  EXPECT_TRUE(ekf.healthy());

  const Vector<4> q_omega(0, WX, 0, WZ);
  Pose pose;
  for (int i = 1; i <= N; ++i) {
    for (int j = 1; j <= 4; j++) {
      static constexpr Scalar DT4 = DT / 4.0;
      state.t += DT4;
      state.qx += (0.5 * DT4 * Q_left(state.q()) * q_omega).eval();
      state.linearize();
      const ImuSample sample{state.t, -state.R().transpose() * GVEC,
                             Vector<3>(WX, 0, WZ)};
      EXPECT_TRUE(ekf.addImu(sample));
    }

    pose.t = state.t;
    pose.attitude = state.q();
    pose.position = state.p;
    EXPECT_TRUE(ekf.addPose(pose));
    EXPECT_TRUE(ekf.healthy());
    const QuadState latest = ekf.getRecent();
    EXPECT_TRUE((latest.p - pose.position).isZero(1e-3))
      << "At time:   " << pose.t << std::endl
      << "Actual:    " << latest.p.transpose() << std::endl
      << "Expected:  " << pose.position.transpose() << std::endl
      << latest << std::endl;
    EXPECT_TRUE((latest.q().coeffs() - pose.attitude.coeffs()).isZero(1e-3))
      << "At time:   " << pose.t << std::endl
      << "Actual:    " << latest.q().coeffs().transpose() << std::endl
      << "Expected:  " << pose.attitude.coeffs().transpose() << std::endl
      << latest << std::endl;
  }
}

// Testing noisy combined linear and rotational motion
TEST(EkfPose, NoisyMotion) {
  // Characteristics for gaussian noise.
  static constexpr Scalar std_pos = 0.003;   // 5mm position noise
  static constexpr Scalar std_att = 0.002;   // ~1 degeree attitude noise
  static constexpr Scalar std_acc = 0.2;     // 0.5 m/s^2 IMU acc noise
  static constexpr Scalar std_omega = 0.05;  // 0.1 m/s^2 IMU acc noise

  // Setup normal distribution.
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<> dist_pos(0.0, std_pos);
  std::normal_distribution<> dist_att(0.0, std_att);
  std::normal_distribution<> dist_acc(0.0, std_acc);
  std::normal_distribution<> dist_omega(0.0, std_omega);

  // Setup filter.
  std::shared_ptr<EkfParameters> params = std::make_shared<EkfParameters>();
  Quadrotor quad(M, L);
  Ekf ekf(quad, params);
  QuadState state;
  state.setZero();
  state.v.x() = VX;
  state.w.x() = WX;

  EXPECT_FALSE(ekf.healthy());
  EXPECT_TRUE(ekf.initialize(state));
  EXPECT_TRUE(ekf.healthy());

  Pose pose{0.0, Vector<3>::Zero(), Quaternion(1, 0, 0, 0)};
  const Vector<4> q_omega(0, WX, 0, 0);
  for (int i = 1; i <= N; ++i) {
    for (int j = 1; j <= 4; j++) {
      static constexpr Scalar DT4 = DT / 4.0;
      state.t += DT4;
      state.p += DT4 * state.v;
      state.qx += (0.5 * DT4 * Q_left(state.q()) * q_omega).eval();
      state.linearize();
      const Vector<3> noise_acc(dist_acc(gen), dist_acc(gen), dist_acc(gen));
      const Vector<3> noise_omega(dist_omega(gen), dist_omega(gen),
                                  dist_omega(gen));
      const ImuSample sample{state.t, -state.R().transpose() * GVEC + noise_acc,
                             Vector<3>(WX, 0, 0) + noise_omega};
      EXPECT_TRUE(ekf.addImu(sample));
    }

    pose.t = state.t;
    pose.position = state.p;
    const Quaternion q = state.q();

    // Adding noise
    const Vector<3> noise_position(dist_pos(gen), dist_pos(gen), dist_pos(gen));
    const Vector<3> noise_attitude(dist_att(gen), dist_att(gen), dist_att(gen));
    pose.position += noise_position;
    Quaternion qn;
    qn.vec() = noise_attitude;
    qn.w() = sqrt(1.0 - qn.vec().norm());
    pose.attitude = q * qn;

    EXPECT_TRUE(ekf.addPose(pose));
    EXPECT_TRUE(ekf.healthy());
  }

  QuadState estimated;
  EXPECT_TRUE(ekf.getRecent(&estimated));
  QuadState expected;
  expected.setZero();
  expected.t = (Scalar)N * DT;
  const Quaternion q(Eigen::AngleAxis(expected.t * WX, Vector<3>::UnitX()));
  expected.p.x() = (Scalar)N * DT * VX;
  expected.v.x() = VX;
  expected.q(q);
  expected.w.x() = WX;
  EXPECT_TRUE(estimated.p.isApprox(expected.p, 1e-2))
    << "Actual:    " << estimated << std::endl
    << "Expected:  " << expected << std::endl;
  EXPECT_TRUE(estimated.qx.isApprox(expected.qx, 1e-2))
    << "Actual:    " << estimated << std::endl
    << "Expected:  " << expected << std::endl;
  ekf.printTimings();
}
