#include "agilib/reference/trajectory_reference/polynomial_trajectory.hpp"
#include "benchmark/benchmark.h"

using namespace agi;

void ClosedFormMinJerkTrajectoryGeneration(benchmark::State& bench) {
  QuadState start_state;
  start_state.setZero();
  start_state.t = 0.0;

  QuadState end_state;
  end_state.setZero();
  end_state.p = Vector<3>(1.0, 1.0, 1.0);
  end_state.t = 355.0 / 113.0;

  for (auto _ : bench) {
    benchmark::DoNotOptimize(
      ClosedFormMinJerkTrajectory(start_state, end_state));
  }
}
BENCHMARK(ClosedFormMinJerkTrajectoryGeneration);

void MinJerkTrajectoryGeneration(benchmark::State& bench) {
  QuadState start_state;
  start_state.setZero();
  start_state.t = 0.0;

  QuadState end_state;
  end_state.setZero();
  end_state.p = Vector<3>(1.0, 1.0, 1.0);
  end_state.t = 355.0 / 113.0;

  for (auto _ : bench) {
    benchmark::DoNotOptimize(MinJerkTrajectory(start_state, end_state));
  }
}
BENCHMARK(MinJerkTrajectoryGeneration);

bool closedFormMinJerkAxisGeneration(const Vector<3>& start_pva,
                                     const Vector<3>& end_pva,
                                     const Scalar start_t,
                                     const Scalar duration) {
  ClosedFormMinJerkAxis traj_axis;
  traj_axis.scale(start_t, duration);
  traj_axis.addConstraint(start_t, start_pva);
  traj_axis.addConstraint(start_t + duration, end_pva);
  traj_axis.solve();
  return true;
}

void ClosedFormMinJerkTrajectoryAxisGeneration(benchmark::State& bench) {
  const Vector<3> start_pva(0.0, 0.0, 0.0);
  const Scalar start_t = 0.0;

  const Vector<3> end_pva(1.0, 3.0, 2.0);
  const Scalar duration = 355.0 / 113.0;

  for (auto _ : bench) {
    benchmark::DoNotOptimize(
      closedFormMinJerkAxisGeneration(start_pva, end_pva, start_t, duration));
  }
}
BENCHMARK(ClosedFormMinJerkTrajectoryAxisGeneration);

bool minJerkAxisGeneration(const Vector<3>& start_pva, const Vector<3>& end_pva,
                           const Scalar start_t, const Scalar duration) {
  Polynomial<> traj_axis(11, Vector<3>(0, 0, 1), -1);
  traj_axis.scale(start_t, duration);
  traj_axis.addConstraint(start_t, start_pva);
  traj_axis.addConstraint(start_t + duration, end_pva);
  traj_axis.solve();
  return true;
}

void MinJerkTrajectoryAxisGeneration(benchmark::State& bench) {
  const Vector<3> start_pva(0.0, 0.0, 0.0);
  const Scalar start_t = 0.0;

  const Vector<3> end_pva(1.0, 3.0, 2.0);
  const Scalar duration = 355.0 / 113.0;

  for (auto _ : bench) {
    benchmark::DoNotOptimize(
      minJerkAxisGeneration(start_pva, end_pva, start_t, duration));
  }
}
BENCHMARK(MinJerkTrajectoryAxisGeneration);
