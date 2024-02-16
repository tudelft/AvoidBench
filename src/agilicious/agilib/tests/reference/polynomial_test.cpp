#include <gtest/gtest.h>

#include "agilib/reference/trajectory_reference/polynomial_trajectory.hpp"

using namespace agi;

class PolynomialTrajectoryTester : public PolynomialTrajectory<Polynomial<>> {
 public:
  using PolynomialTrajectory::PolynomialTrajectory;
  const Polynomial<>& getXprob() { return x_; }
  const Polynomial<>& getYprob() { return y_; }
  const Polynomial<>& getZprob() { return z_; }
  const Polynomial<>& getYawprob() { return yaw_; }
};

class ClosedFormTrajectoryTester : public ClosedFormMinJerkTrajectory {
 public:
  using ClosedFormMinJerkTrajectory::ClosedFormMinJerkTrajectory;
  const ClosedFormMinJerkAxis& getXprob() { return x_; }
  const ClosedFormMinJerkAxis& getYprob() { return y_; }
  const ClosedFormMinJerkAxis& getZprob() { return z_; }
  const ClosedFormMinJerkAxis& getYawprob() { return yaw_; }
};

static constexpr int order = 11;

TEST(Polynomial, CreatePotent) {
  Polynomial<> poly(order, Vector<>::Ones(order));
  const Array<> potent = poly.exponents();
  const Array<> alpha = poly.alpha();

  for (int i = 0; i <= order; ++i)
    for (int j = 0; j <= order; ++j) {
      // Check potent.
      EXPECT_EQ(potent(i, j), std::max((Scalar)(j - i), 0.0));

      // Check alpha.
      EXPECT_EQ(alpha(i, j),
                [&]() {
                  Scalar x = 1.0;
                  for (int k = j - i + 1; k <= j; k++) x *= (Scalar)k;
                  return x;
                }())
        << "where i=" << i << " and j=" << j;
    }
}

TEST(Polynomial, SingleAxisTest) {
  static constexpr Scalar t0 = 1.0;
  static constexpr Scalar t1 = 4.0;
  static constexpr Scalar dt = 1e-4;
  static constexpr Scalar p0 = 1.0;
  static constexpr Scalar p1 = p0 + 10.0;
  static constexpr Scalar v = 1.0;
  static constexpr Scalar tol = 1e-5;

  Polynomial<> poly(order, Vector<4>(0, 0, 0, 1));
  poly.scale(t0, t1 - t0);

  const Vector<> c0 = Vector<4>(p0, v, 0, 0);
  const Vector<> c1 = Vector<4>(p1, v, 0, 0);
  poly.addConstraint(t0, c0);
  poly.addConstraint(t1, c1);
  EXPECT_TRUE(poly.solve());

  EXPECT_NEAR(poly.coeffs()(0), p0, tol);
  EXPECT_NEAR(poly.alpha().row(0) * poly.coeffs(), p1, tol);
  EXPECT_NEAR(poly(t0), p0, tol);
  EXPECT_NEAR(poly(t1), p1, tol);

  Vector<> x = Vector<>::Zero(4);
  poly.eval(t0, x);

  EXPECT_NEAR(x(0), p0, tol);
  EXPECT_NEAR(x(1), v, tol);

  EXPECT_TRUE(poly.solved());
  for (Scalar t = t0 + dt; t < t1; t += dt) {
    x.segment<3>(0) += dt * x.segment<3>(1);
    Vector<> x_comp = Vector<>::Zero(4);
    poly.eval(t, x_comp);
    EXPECT_NEAR(x(0), x_comp(0), tol) << "at " << t;
    EXPECT_NEAR(x(1), x_comp(1), tol) << "at " << t;
    EXPECT_NEAR(x(2), x_comp(2), tol) << "at " << t;
    x = x_comp;
  }
}

TEST(Polynomial, MinSnapStateToState) {
  QuadState start_state;
  start_state.setZero();
  start_state.t = 1.0;
  QuadState end_state;
  end_state.setZero();
  end_state.p.x() = 1.0;
  end_state.t = 3.86786278923098;
  QuadState mid_state;
  mid_state.t = start_state.t + 0.5 * (end_state.t - start_state.t);
  mid_state.p = 0.5 * (start_state.p + end_state.p);

  PolynomialTrajectoryTester traj(start_state, end_state);

  Setpoint start_point = traj.getStartSetpoint();
  Setpoint end_point = traj.getEndSetpoint();
  Setpoint mid_point = traj.getSetpoint(mid_state);

  EXPECT_TRUE(start_point.state.isApprox(start_state, 1e-6));
  EXPECT_TRUE(end_point.state.isApprox(end_state, 1e-6));
  EXPECT_TRUE(mid_point.state.p.isApprox(mid_state.p, 1e-3))
    << "Expected: " << mid_state.p.transpose() << std::endl
    << "Actual:   " << mid_point.state.p.transpose() << std::endl;
}

TEST(ClosedFormMinJerkAxis, MinJerkPosVelAccFixed) {
  QuadState start_state;
  start_state.setZero();
  start_state.t = 1.0;
  QuadState end_state;
  end_state.setZero();
  end_state.p = Vector<3>(1.0, 1.0, 1.0);
  end_state.t = 355.0 / 113.0;
  QuadState mid_state;
  mid_state.t = start_state.t + 0.5 * (end_state.t - start_state.t);
  mid_state.p = 0.5 * (start_state.p + end_state.p);

  ClosedFormTrajectoryTester traj(start_state, end_state);

  Setpoint start_point = traj.getSetpoint(start_state);
  Setpoint end_point = traj.getSetpoint(end_state);
  Setpoint mid_point = traj.getSetpoint(mid_state);

  EXPECT_TRUE((start_point.state.p - start_state.p).isZero(1e-3))
    << "Expected: " << start_state.p.transpose() << std::endl
    << "Actual:   " << start_point.state.p.transpose() << std::endl;
  EXPECT_TRUE((start_point.state.v - start_state.v).isZero(1e-3))
    << "Expected: " << start_state.v.transpose() << std::endl
    << "Actual:   " << start_point.state.v.transpose() << std::endl;
  EXPECT_TRUE((start_point.state.a - start_state.a).isZero(1e-3))
    << "Expected: " << start_state.a.transpose() << std::endl
    << "Actual:   " << start_point.state.a.transpose() << std::endl;
  EXPECT_TRUE((start_point.state.a - start_state.a).isZero(1e-3));

  EXPECT_TRUE((end_point.state.p - end_state.p).isZero(1e-3))
    << "Expected: " << end_state.p.transpose() << std::endl
    << "Actual:   " << end_point.state.p.transpose() << std::endl;
  EXPECT_TRUE((end_point.state.v - end_state.v).isZero(1e-3))
    << "Expected: " << end_state.v.transpose() << std::endl
    << "Actual:   " << end_point.state.v.transpose() << std::endl;
  EXPECT_TRUE((end_point.state.a - end_state.a).isZero(1e-3))
    << "Expected: " << end_state.a.transpose() << std::endl
    << "Actual:   " << end_point.state.a.transpose() << std::endl;

  EXPECT_TRUE((mid_point.state.p - mid_state.p).isZero(1e-3))
    << "Expected: " << mid_state.p.transpose() << std::endl
    << "Actual:   " << mid_point.state.p.transpose() << std::endl;
}


// Left for future implementations

// TEST(ClosedFormMinJerkAxis, MinJerkPosVelFixed) {
//   QuadState start_state;
//   start_state.setZero();
//   start_state.t = 1.0;
//   QuadState end_state;
//   end_state.x.setConstant(NAN);
//   end_state.p = Vector<3>(1.0, 1.0, 1.0);
//   end_state.v = Vector<3>(2.0, 2.0, 1.0);
//   end_state.t = 355.0 / 113.0;

//   ClosedFormTrajectoryTester traj(start_state, end_state);

//   Setpoint start_point;
//   traj.getSetpoint(start_state.t, NAN, &start_point);
//   Setpoint end_point;
//   traj.getSetpoint(end_state.t, NAN, &end_point);

//   EXPECT_TRUE((start_point.state.p - start_state.p).isZero(1e-3));
//   EXPECT_TRUE((start_point.state.v - start_state.v).isZero(1e-3));

//   EXPECT_TRUE((end_point.state.p - end_state.p).isZero(1e-3));
//   EXPECT_TRUE((end_point.state.v - end_state.v).isZero(1e-3));
// }

// TEST(ClosedFormMinJerkAxis, MinJerkPosFixed) {
//   QuadState start_state;
//   start_state.setZero();
//   start_state.t = 1.0;
//   QuadState end_state;
//   end_state.x.setConstant(NAN);
//   end_state.p = Vector<3>(1.0, 1.0, 1.0);
//   end_state.t = 355.0 / 113.0;

//   ClosedFormTrajectoryTester traj(start_state, end_state);

//   Setpoint start_point;
//   traj.getSetpoint(start_state.t, NAN, &start_point);
//   Setpoint end_point;
//   traj.getSetpoint(end_state.t, NAN, &end_point);

//   EXPECT_TRUE((start_point.state.p - start_state.p).isZero(1e-3));

//   EXPECT_TRUE((end_point.state.p - end_state.p).isZero(1e-3));
// }

// TEST(ClosedFormMinJerkAxis, MinJerkPosAccFixed) {
//   QuadState start_state;
//   start_state.setZero();
//   start_state.t = 1.0;
//   QuadState end_state;
//   end_state.x.setConstant(NAN);
//   end_state.p = Vector<3>(1.0, 1.0, 1.0);
//   end_state.a = Vector<3>(2.0, 1.0, 0.0);
//   end_state.t = 355.0 / 113.0;

//   ClosedFormTrajectoryTester traj(start_state, end_state);

//   Setpoint start_point;
//   traj.getSetpoint(start_state.t, NAN, &start_point);
//   Setpoint end_point;
//   traj.getSetpoint(end_state.t, NAN, &end_point);

//   EXPECT_TRUE((start_point.state.p - start_state.p).isZero(1e-3));
//   EXPECT_TRUE((start_point.state.a - start_state.a).isZero(1e-3));

//   EXPECT_TRUE((end_point.state.p - end_state.p).isZero(1e-3));
//   EXPECT_TRUE((end_point.state.a - end_state.a).isZero(1e-3));
// }
