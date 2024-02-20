#include <gtest/gtest.h>

#include "agilib/math/gravity.hpp"
#include "agilib/reference/trajectory_reference/polynomial_trajectory.hpp"
#include "agilib/sampler/time_based/time_sampler.hpp"

using namespace agi;

template<typename Reference>
ReferenceVector createTrajectory(const int rounds, const Scalar speed) {
  ReferenceVector references;

  QuadState start_state;
  start_state.t = 0.0;
  start_state.setZero();
  start_state.p = Vector<3>(-4.0, 5.5, 1.2);

  const Matrix<> waypoints =
    (Matrix<>(3, 8) << Vector<3>(-1.1, -1.6, 3.6), Vector<3>(9.2, 6.6, 1.0),
     Vector<3>(9.2, -4.0, 1.2), Vector<3>(-2.5, -6.0, 3.5),
     Vector<3>(-6.0, -6.0, 2.2), Vector<3>(-2.5, -6.0, 0.8),
     Vector<3>(4.75, -0.9, 1.2), Vector<3>(-2.8, 6.8, 1.2))
      .finished();
  const Matrix<> directions =
    (Matrix<>(3, 8) << Vector<3>(2, -1, 0), Vector<3>(1, -1, 0),
     Vector<3>(-1, -1, 0), Vector<3>(-1, 0, 0), Vector<3>(0, 0, -1),
     Vector<3>(1, 0, 0), Vector<3>(0, 1, 0), Vector<3>(-1, -1, 0))
      .finished();

  const Vector<> speed_scale =
    (Vector<>(8) << 1, 1.3, 1.5, 1.2, 0.8, 1.4, 1.5, 1.0).finished();

  QuadState end_state;
  end_state.setZero();
  end_state.p = Vector<3>(-3.5, 1.0, 1.2);

  std::vector<QuadState> states;
  states.push_back(start_state);

  for (int i = 0; i < waypoints.cols(); ++i) {
    const Vector<3> pos = waypoints.col(i);
    const Vector<3> dir = directions.col(i);
    QuadState state;
    state.setZero();
    state.p = pos;
    state.v = speed_scale(i) * speed * dir;
    const Scalar duration = (pos - states.back().p).norm() / state.v.norm();
    state.t = states.back().t + duration;
    states.push_back(state);
  }

  {
    const Scalar duration = (end_state.p - states.back().p).norm() / speed;
    end_state.t = states.back().t + duration;
  }

  states.push_back(end_state);

  for (int i = 0; i < (int)(states.size() - 1); ++i)
    references.push_back(
      std::make_shared<Reference>(states.at(i), states.at(i + 1)));

  return references;
}

enum Continuity : int { POSITION = 0, VELOCITY, ACCELERATION, JERK, SNAP };

void checkTrajectory(const ReferenceVector& references,
                     const Continuity continuity = SNAP) {
  const Scalar dt = 0.0001;
  const Scalar duration =
    references.back()->getEndTime() - references.front()->getStartTime();

  const int n = duration / dt;
  TimeSampler sampler(n, dt);
  SetpointVector setpoints;

  const QuadState query = references.front()->getStartSetpoint().state;
  EXPECT_TRUE(sampler.getAt(query, references, &setpoints));

  const Scalar dt_inv = 1.0 / dt;
  static constexpr Scalar tol = 1e-1;
  for (int i = 0; i < n - 1; ++i) {
    QuadState state0 = setpoints.at(i).state;
    QuadState state1 = setpoints.at(i + 1).state;

    if (continuity >= VELOCITY) {
      const Vector<3> vel = dt_inv * (state1.p - state0.p);
      const Vector<3> vel_exp = 0.5 * (state0.v + state1.v);
      EXPECT_LT((vel - vel_exp).norm(), 1e0 * tol)
        << "at " << state0.t << std::endl
        << "Analytical: " << vel_exp.transpose() << std::endl
        << "Numerical:  " << vel.transpose() << std::endl;
    }

    if (continuity >= ACCELERATION) {
      const Vector<3> acc = dt_inv * (state1.v - state0.v);
      const Vector<3> acc_exp = 0.5 * (state0.a + state1.a);
      EXPECT_LT((acc - acc_exp).norm(), 1e1 * tol)
        << "at " << state0.t << std::endl
        << "Analytical: " << acc_exp.transpose() << std::endl
        << "Numerical:  " << acc.transpose() << std::endl;
    }

    if (continuity >= JERK) {
      const Vector<3> jerk = dt_inv * (state1.a - state0.a);
      const Vector<3> jerk_exp = 0.5 * (state0.j + state1.j);
      EXPECT_LT((jerk - jerk_exp).norm(), 1e2 * tol)
        << "at " << state0.t << std::endl
        << "Analytical: " << jerk_exp.transpose() << std::endl
        << "Numerical:  " << jerk.transpose() << std::endl;
    }

    if (continuity >= SNAP) {
      const Vector<3> snap = dt_inv * (state1.j - state0.j);
      const Vector<3> snap_exp = 0.5 * (state0.s + state1.s);
      EXPECT_LT((snap - snap_exp).norm(), 1e3 * tol)
        << "at " << state0.t << std::endl
        << "Analytical: " << snap_exp.transpose() << std::endl
        << "Numerical:  " << snap.transpose() << std::endl;
    }
  }
}

class DemoPolynomialTrajectoriesParams
  : public testing::TestWithParam<std::tuple<Scalar>> {};

static constexpr int rounds = 4;

TEST_P(DemoPolynomialTrajectoriesParams, MinJerkTrajectory) {
  const Scalar speed = std::get<0>(GetParam());

  ReferenceVector trajs = createTrajectory<MinJerkTrajectory>(rounds, speed);
  checkTrajectory(trajs, ACCELERATION);
}

TEST_P(DemoPolynomialTrajectoriesParams, MinSnapTrajectory) {
  const Scalar speed = std::get<0>(GetParam());

  ReferenceVector trajs = createTrajectory<MinSnapTrajectory>(rounds, speed);
  checkTrajectory(trajs, JERK);
}

TEST_P(DemoPolynomialTrajectoriesParams, ClosedFormMinJerkTrajectory) {
  const Scalar speed = std::get<0>(GetParam());

  ReferenceVector trajs =
    createTrajectory<ClosedFormMinJerkTrajectory>(rounds, speed);
  checkTrajectory(trajs, ACCELERATION);
}

INSTANTIATE_TEST_SUITE_P(DemoPolynomialTrajectories,
                         DemoPolynomialTrajectoriesParams,
                         ::testing::Values(6, 8, 10, 12));


TEST(DemoPolynomialTrajectories, Rescaling) {
  QuadState start_state;
  start_state.t = 0.0;
  start_state.p = Vector<3>(-4.0, 5.5, 1.2);
  start_state.v.setZero();
  start_state.a.setZero();
  start_state.q(Quaternion(1, 0, 0, 0));

  const Matrix<> waypoints =
    (Matrix<>(3, 8) << Vector<3>(-1.1, -1.6, 3.6), Vector<3>(9.2, 6.6, 1.0),
     Vector<3>(9.2, -4.0, 1.2), Vector<3>(-2.5, -6.0, 3.5),
     Vector<3>(-6.0, -6.0, 2.2), Vector<3>(-2.5, -6.0, 0.8),
     Vector<3>(4.75, -0.9, 1.2), Vector<3>(-2.8, 6.8, 1.2))
      .finished();

  QuadState end_state;
  end_state.t = 1.0 + waypoints.cols();
  end_state.p = Vector<3>(-3.5, 1.0, 1.2);
  end_state.v.setZero();
  end_state.a.setZero();
  end_state.q(Quaternion(1, 0, 0, 0));

  std::vector<QuadState> states;
  states.push_back(start_state);
  for (int i = 0; i < waypoints.cols(); ++i) {
    QuadState state;
    state.t = 1.0 + i;
    state.p = waypoints.col(i);
    states.push_back(state);
  }

  states.push_back(end_state);

  Quadrotor quad{1.0, 0.15};
  quad.thrust_min_ = 0.0;
  quad.thrust_max_ = 3.0 * G / 4.0 * quad.m_;
  std::printf("Quadrotor max thrust: %1.3f m/s^2\n",
              quad.collective_thrust_max());

  PolynomialTrajectory traj(states, Vector<4>(0, 0, 0, 1), 23);
  EXPECT_TRUE(traj.valid());

  const Scalar t_acc = traj.findTimeMaxAcc();
  const Scalar t_omega = traj.findTimeMaxOmega();

  const Vector<> max_acc_v = traj.evalTranslation(t_acc, 2);
  const Scalar max_acc = (max_acc_v - GVEC).norm();
  QuadState state_max_omega = traj.getState(t_omega);

  std::printf("Time range [%1.3f, %1.3f]\n", traj.getStartTime(),
              traj.getEndTime());
  std::printf("Time max acc at   %1.3f  with %1.3f m/s^2\n", t_acc, max_acc);
  std::printf("Time max omega at %1.3f  with %1.3f rad/s\n", t_omega,
              state_max_omega.w.array().abs().maxCoeff());

  static constexpr Scalar dt = 0.1;
  const int N = (states.back().t - states.front().t) / dt;
  Vector<> accs = Vector<>::Zero(N);
  Vector<> omegas = Vector<>::Zero(N);
  for (int i = 0; i < N; ++i) {
    const Scalar t = states.front().t + i * dt;
    accs(i) = (traj.evalTranslation(t, 2) - GVEC).norm();
  }

  const Scalar acc_max_sampled = accs.maxCoeff();
  std::printf("Sampled max acc:   %1.3f\n", acc_max_sampled);

  traj.scaleToLimits(quad, 100, 1e-6);

  const Scalar t_acc_scaled = traj.findTimeMaxAcc();
  const Scalar t_omega_scaled = traj.findTimeMaxOmega();

  const Vector<> max_acc_v_scaled = traj.evalTranslation(t_acc_scaled, 2);
  const Scalar max_acc_scaled = (max_acc_v_scaled - GVEC).norm();
  const QuadState state_max_omega_scaled = traj.getState(t_omega_scaled);

  std::printf("Scaled trajectory time: [%1.3f, %1.3f] over %1.3fs\n",
              traj.getStartTime(), traj.getEndTime(),
              traj.getEndTime() - traj.getStartTime());
  std::printf("Time max acc at   %1.3f  with %1.3f m/s^2\n", t_acc_scaled,
              max_acc_scaled);
  std::printf("Time max omega at %1.3f  with %1.3f rad/s\n", t_omega_scaled,
              state_max_omega_scaled.w.array().abs().maxCoeff());
}
