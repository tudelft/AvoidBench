#include <gtest/gtest.h>

#include "agilib/math/integrator_base.hpp"
#include "agilib/math/integrator_euler.hpp"
#include "agilib/math/integrator_rk4.hpp"
#include "agilib/types/quad_state.hpp"
#include "agilib/types/quadrotor.hpp"

using namespace agi;

static constexpr Scalar m = 1.0;
static constexpr Scalar l = 0.25;

TEST(Integrators, ManualEulerAccelerationCheck) {
  static constexpr Scalar dt = 1.0;
  // Using lower tolerance for check because of accuracy of forward Euler.
  static constexpr Scalar tol = 1e-3;

  QuadState initial_state;
  initial_state.setZero();

  const Quadrotor quad(m, l);

  IntegratorEuler euler(quad.getDynamicsFunction());

  initial_state.a = Vector<3>::Random();

  QuadState expected_state(initial_state);
  expected_state.p = initial_state.p + dt * dt / 2.0 * initial_state.a;
  expected_state.v = initial_state.v + dt * initial_state.a;

  QuadState final_state;

  EXPECT_TRUE(euler.integrate(initial_state.x, dt, final_state.x));
  EXPECT_TRUE(final_state.x.isApprox(expected_state.x, tol))
    << "expected state:   " << expected_state.x.transpose() << "\n"
    << "integrated state: " << final_state.x.transpose() << "\n";
}

TEST(Integrators, ManualRungeKuttaAccelerationCheck) {
  static constexpr Scalar dt = 1.0;

  QuadState initial_state;
  initial_state.setZero();

  const Quadrotor quad(m, l);

  IntegratorRK4 rungekutta(quad.getDynamicsFunction());

  initial_state.a = Vector<3>::Random();

  QuadState expected_state(initial_state);
  expected_state.p = initial_state.p + dt * dt / 2.0 * initial_state.a;
  expected_state.v = initial_state.v + dt * initial_state.a;

  QuadState final_state;

  EXPECT_TRUE(rungekutta.integrate(initial_state.x, dt, final_state.x));
  EXPECT_TRUE(final_state.x.isApprox(expected_state.x))
    << "expected state:   " << expected_state.x.transpose() << "\n"
    << "integrated state: " << final_state.x.transpose() << "\n";
}

TEST(Integrators, QuadStateInterface) {
  static constexpr Scalar dt = 1.0;

  QuadState initial_state;
  initial_state.setZero();

  const Quadrotor quad(m, l);

  QuadState int_euler;
  QuadState int_rungekutta;
  int_euler.t = dt;
  int_rungekutta.t = dt;

  IntegratorEuler euler(quad.getDynamicsFunction());
  IntegratorRK4 rungekutta(quad.getDynamicsFunction());

  EXPECT_TRUE(euler.integrate(initial_state, &int_euler));
  EXPECT_TRUE(rungekutta.integrate(initial_state, &int_rungekutta));

  EXPECT_TRUE(int_euler.x.isApprox(initial_state.x));
  EXPECT_TRUE(int_rungekutta.x.isApprox(initial_state.x));

  int_euler.t = -0.1;
  int_rungekutta.t = -0.1;

  EXPECT_FALSE(euler.integrate(initial_state, &int_euler));
  EXPECT_FALSE(rungekutta.integrate(initial_state, &int_rungekutta));
}

TEST(Integrators, CheckEulerAgainstRungeKutta) {
  static constexpr int N = 16;  // Test not too often for speed in debug mode.
  static constexpr Scalar dt = 0.1;
  // Using lower tolerance for check because of accuracy of forward Euler.
  static constexpr Scalar tol = 1e-2;

  const Quadrotor quad(m, l);
  const IntegratorEuler euler(quad.getDynamicsFunction(), 1e-3);
  const IntegratorRK4 rungekutta(quad.getDynamicsFunction(), 1e-3);

  for (int trials = 0; trials < N; ++trials) {
    QuadState initial_state(0.0, Vector<QuadState::SIZE>::Random());
    initial_state.qx.normalize();

    QuadState int_euler;
    QuadState int_rungekutta;

    EXPECT_TRUE(euler.integrate(initial_state.x, dt, int_euler.x));
    EXPECT_TRUE(rungekutta.integrate(initial_state.x, dt, int_rungekutta.x));
    EXPECT_TRUE(int_euler.x.isApprox(int_rungekutta.x, tol))
      << "Euler intergrated:\n"
      << int_euler.x.transpose() << std::endl
      << "RungeKutta intergrated:\n"
      << int_rungekutta.x.transpose() << std::endl;
  }
}
