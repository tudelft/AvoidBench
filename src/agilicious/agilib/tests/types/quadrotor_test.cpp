#include "agilib/types/quadrotor.hpp"

#include <gtest/gtest.h>

#include "agilib/math/math.hpp"
#include "agilib/types/quad_state.hpp"

using namespace agi;

static constexpr Scalar m = 1.0;
static constexpr Scalar l = 0.25;

TEST(Quadrotor, Constructor) {
  const Matrix<3, 4> t_BM =
    l * sqrt(0.5) *
    (Matrix<3, 4>() << 1, -1, -1, 1, -1, 1, -1, 1, 0, 0, 0, 0).finished();

  // Compute default inertia from formula for homogenious cuboid.
  const Scalar X = 2.0 * sqrt(0.5) * l;
  const Scalar Y = 2.0 * sqrt(0.5) * l;
  const Scalar Z = 0.5 * l;
  const Matrix<3, 3> J =
    m / 12.0 *
    Vector<3>(Y * Y + Z * Z, X * X + Z * Z, X * X + Y * Y).asDiagonal();

  const Quadrotor quad(m, l);

  const Quadrotor quad_copy(quad);

  EXPECT_EQ(quad_copy.m_, m);
  EXPECT_TRUE(quad_copy.t_BM_.isApprox(t_BM));
  EXPECT_TRUE(quad_copy.J_.isApprox(J));
  EXPECT_TRUE(quad_copy.J_inv_.isApprox(J.inverse()));
}

TEST(Quadrotor, dynamics) {
  const Quadrotor quad(m, l);

  QuadState hover;
  hover.setZero();

  Vector<> derivative(hover.x);
  QuadState derivative_state;
  derivative_state.setZero();

  EXPECT_TRUE(quad.dynamics(hover.x, derivative));
  EXPECT_TRUE(quad.dynamics(hover, &derivative_state));

  EXPECT_TRUE(derivative.isZero());
  EXPECT_TRUE(derivative_state.x.isZero());

  static constexpr int N = 128;

  for (int trail = 0; trail < N; ++trail) {
    // Generate a random state.
    QuadState random_state;
    random_state.x = Vector<QuadState::SIZE>::Random();
    random_state.qx.normalize();

    // Get the differential.
    EXPECT_TRUE(quad.dynamics(random_state, &derivative_state));

    // Compute it manually.
    QuadState derivative_manual;
    derivative_manual.setZero();

    const Quaternion q_omega(0, random_state.w.x(), random_state.w.y(),
                             random_state.w.z());
    derivative_manual.p = random_state.v;
    derivative_manual.qx = 0.5 * Q_right(q_omega) * random_state.qx;
    derivative_manual.v = random_state.a;
    derivative_manual.w =
      quad.J_inv_ *
      (random_state.tau - random_state.w.cross(quad.J_ * random_state.w));

    // Compare the derivatives.
    EXPECT_TRUE(derivative_state.x.isApprox(derivative_manual.x));
  }
}

TEST(Quadrotor, Jacobian) {
  const Quadrotor quad(m, l);

  QuadState state_nominal;
  state_nominal.setZero();

  static constexpr int N = 128;
  static constexpr Scalar dx = 1e-6;
  static constexpr Scalar dx_inv2 = 0.5 / dx;
  static constexpr Scalar tol = 1e-6;

  for (int trial = 0; trial < N; ++trial) {
    state_nominal.x = Vector<QuadState::SIZE>::Random();

    Matrix<QuadState::SIZE, QuadState::SIZE> jacobian_analytic;
    Matrix<QuadState::SIZE, QuadState::SIZE> jacobian_numeric;
    quad.jacobian(state_nominal.x, jacobian_analytic);

    for (int i = 0; i < QuadState::SIZE; ++i) {
      Vector<QuadState::SIZE> dxp(state_nominal.x);
      Vector<QuadState::SIZE> dxn(state_nominal.x);

      dxp(i) += dx;
      dxn(i) -= dx;

      Vector<QuadState::SIZE> derivative_p;
      Vector<QuadState::SIZE> derivative_n;

      quad.dynamics(dxp, derivative_p);
      quad.dynamics(dxn, derivative_n);

      jacobian_numeric.col(i) = dx_inv2 * (derivative_p - derivative_n);
    }

    for (int i = 0; i < QuadState::SIZE; ++i) {
      for (int j = 0; j < QuadState::SIZE; ++j) {
        EXPECT_NEAR(jacobian_analytic(i, j), jacobian_numeric(i, j), tol)
          << "in the Jacobian at [" << i << ", " << j << "]\n";
      }
    }
  }
}

TEST(Quadrotor, VectorReference) {
  const Quadrotor quad(m, l);

  static constexpr int N = 128;
  Matrix<QuadState::SIZE, N> states = Matrix<QuadState::SIZE, N>::Random();
  Matrix<QuadState::SIZE, N> states_const =
    Matrix<QuadState::SIZE, N>::Random();
  Matrix<QuadState::SIZE, N> derivates;

  for (int i = 0; i < N; ++i) {
    QuadState derivate;
    EXPECT_TRUE(quad.dynamics(states.col(i), derivates.col(i)));
    EXPECT_TRUE(quad.dynamics(QuadState(0.0, states.col(i)), &derivate));
    EXPECT_TRUE(quad.dynamics(states_const.col(i), derivates.col(i)));
  }
}
