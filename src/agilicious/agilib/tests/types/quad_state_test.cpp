#include "agilib/types/quad_state.hpp"

#include <gtest/gtest.h>

using namespace agi;

TEST(QuadState, Constructor) {
  QuadState state;
  EXPECT_TRUE(state.x.hasNaN());

  QuadState zero_static_state(0.0, Vector<QuadState::SIZE>::Zero());
  EXPECT_TRUE(zero_static_state.x.allFinite());

  QuadState zero_dynamic_state(0.0, Vector<>::Zero(QuadState::SIZE));
  EXPECT_TRUE(zero_dynamic_state.x.allFinite());
}

TEST(QuadState, Accessors) {
  const int N = QuadState::SIZE;
  Vector<> x(N);
  for (int i = 0; i < QuadState::SIZE; ++i) x(i) = i;

  QuadState state(0.0, x);

  EXPECT_EQ(state.p(0), x(0));
  EXPECT_EQ(state.p(1), x(1));
  EXPECT_EQ(state.p(2), x(2));
  EXPECT_EQ(state.qx(0), x(3));
  EXPECT_EQ(state.qx(1), x(4));
  EXPECT_EQ(state.qx(2), x(5));
  EXPECT_EQ(state.qx(3), x(6));
  EXPECT_EQ(state.v(0), x(7));
  EXPECT_EQ(state.v(1), x(8));
  EXPECT_EQ(state.v(2), x(9));
  EXPECT_EQ(state.w(0), x(10));
  EXPECT_EQ(state.w(1), x(11));
  EXPECT_EQ(state.w(2), x(12));
  EXPECT_EQ(state.a(0), x(13));
  EXPECT_EQ(state.a(1), x(14));
  EXPECT_EQ(state.a(2), x(15));
  EXPECT_EQ(state.tau(0), x(16));
  EXPECT_EQ(state.tau(1), x(17));
  EXPECT_EQ(state.tau(2), x(18));
  EXPECT_EQ(state.j(0), x(19));
  EXPECT_EQ(state.j(1), x(20));
  EXPECT_EQ(state.j(2), x(21));
  EXPECT_EQ(state.s(0), x(22));
  EXPECT_EQ(state.s(1), x(23));
  EXPECT_EQ(state.s(2), x(24));
  EXPECT_EQ(state.bw(0), x(25));
  EXPECT_EQ(state.bw(1), x(26));
  EXPECT_EQ(state.bw(2), x(27));
  EXPECT_EQ(state.ba(0), x(28));
  EXPECT_EQ(state.ba(1), x(29));
  EXPECT_EQ(state.ba(2), x(30));
  EXPECT_EQ(state.mot(0), x(31));
  EXPECT_EQ(state.mot(1), x(32));
  EXPECT_EQ(state.mot(2), x(33));
  EXPECT_EQ(state.mot(3), x(34));
  EXPECT_EQ(state.motdes(0), x(35));
  EXPECT_EQ(state.motdes(1), x(36));
  EXPECT_EQ(state.motdes(2), x(37));
  EXPECT_EQ(state.motdes(3), x(38));

  x += Vector<>::Ones(QuadState::SIZE);
  state.p += Vector<3>::Ones();
  state.qx += Vector<4>::Ones();
  state.v += Vector<3>::Ones();
  state.w += Vector<3>::Ones();
  state.a += Vector<3>::Ones();
  state.tau += Vector<3>::Ones();
  state.j += Vector<3>::Ones();
  state.s += Vector<3>::Ones();
  state.bw += Vector<3>::Ones();
  state.ba += Vector<3>::Ones();
  state.mot += Vector<4>::Ones();
  state.motdes += Vector<4>::Ones();

  EXPECT_EQ(state.p(0), x(0));
  EXPECT_EQ(state.p(1), x(1));
  EXPECT_EQ(state.p(2), x(2));
  EXPECT_EQ(state.qx(0), x(3));
  EXPECT_EQ(state.qx(1), x(4));
  EXPECT_EQ(state.qx(2), x(5));
  EXPECT_EQ(state.qx(3), x(6));
  EXPECT_EQ(state.v(0), x(7));
  EXPECT_EQ(state.v(1), x(8));
  EXPECT_EQ(state.v(2), x(9));
  EXPECT_EQ(state.w(0), x(10));
  EXPECT_EQ(state.w(1), x(11));
  EXPECT_EQ(state.w(2), x(12));
  EXPECT_EQ(state.a(0), x(13));
  EXPECT_EQ(state.a(1), x(14));
  EXPECT_EQ(state.a(2), x(15));
  EXPECT_EQ(state.tau(0), x(16));
  EXPECT_EQ(state.tau(1), x(17));
  EXPECT_EQ(state.tau(2), x(18));
  EXPECT_EQ(state.j(0), x(19));
  EXPECT_EQ(state.j(1), x(20));
  EXPECT_EQ(state.j(2), x(21));
  EXPECT_EQ(state.s(0), x(22));
  EXPECT_EQ(state.s(1), x(23));
  EXPECT_EQ(state.s(2), x(24));
  EXPECT_EQ(state.bw(0), x(25));
  EXPECT_EQ(state.bw(1), x(26));
  EXPECT_EQ(state.bw(2), x(27));
  EXPECT_EQ(state.ba(0), x(28));
  EXPECT_EQ(state.ba(1), x(29));
  EXPECT_EQ(state.ba(2), x(30));
  EXPECT_EQ(state.mot(0), x(31));
  EXPECT_EQ(state.mot(1), x(32));
  EXPECT_EQ(state.mot(2), x(33));
  EXPECT_EQ(state.mot(3), x(34));
  EXPECT_EQ(state.motdes(0), x(35));
  EXPECT_EQ(state.motdes(1), x(36));
  EXPECT_EQ(state.motdes(2), x(37));
  EXPECT_EQ(state.motdes(3), x(38));

  EXPECT_TRUE(state.x.isApprox(x));
}

TEST(QuadState, Compare) {
  const int N = QuadState::SIZE;
  Vector<> x(N);
  for (int i = 0; i < QuadState::SIZE; ++i) x(i) = i;

  QuadState state(0.0, x);
  QuadState other_state(0.0, x);

  EXPECT_TRUE(state == other_state);
  state.p += Vector<3>::Ones();
  EXPECT_FALSE(state == other_state);
  other_state.p += Vector<3>::Ones();
  EXPECT_TRUE(state == other_state);
  state.t += 1.0;
  EXPECT_FALSE(state == other_state);
}

TEST(QuadState, Heading) {
  static constexpr int N = 1e4;

  for (int i = 0; i < N; ++i) {
    const Vector<3> angles = M_PI * Vector<3>::Random();
    QuadState state;
    state.setZero();

    state.q(Quaternion(Eigen::AngleAxis(angles.z(), Vector<3>::UnitZ()) *
                       Eigen::AngleAxis(angles.x(), Vector<3>::UnitX())));
    EXPECT_NEAR(state.getYaw(), angles.z(), 1e-3);
  }
}
