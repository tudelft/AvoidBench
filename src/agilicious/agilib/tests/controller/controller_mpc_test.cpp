#include "agilib/controller/mpc/controller_mpc.hpp"

#include <gtest/gtest.h>

#include "agilib/controller/mpc/mpc_params.hpp"
#include "agilib/math/gravity.hpp"
#include "agilib/simulator/quadrotor_simulator.hpp"
#include "agilib/types/command.hpp"
#include "agilib/types/quad_state.hpp"
#include "agilib/types/quadrotor.hpp"
#include "agilib/types/setpoint.hpp"

using namespace agi;

static constexpr Scalar M = 1.5;
static constexpr Scalar L = 0.25;

TEST(MPC, ConstrutorTest) {
  Quadrotor quad(M, L);
  std::shared_ptr<MpcParameters> params = std::make_shared<MpcParameters>();
  MpcController mpc(quad, params);
}

TEST(MPC, StaticTest) {
  Quadrotor quad(M, L);
  std::shared_ptr<MpcParameters> params = std::make_shared<MpcParameters>();
  MpcController mpc(quad, params);

  QuadState hover_state;
  hover_state.setZero();
  Command hover_command;
  hover_command.t = 0.0;
  hover_command.thrusts.setConstant(M * G / 4.0);

  SetpointVector references(20, Setpoint(hover_state, hover_command));
  SetpointVector setpoints;

  for (int i = 0; i < 10; ++i) {
    mpc.getCommand(hover_state, references, &setpoints);
    Command command = setpoints.front().input;
    EXPECT_TRUE(command.thrusts.isApprox(hover_command.thrusts, 1e-3))
      << "Actual:   " << command.thrusts.transpose() << std::endl
      << "Expected: " << hover_command.thrusts.transpose() << std::endl;
    usleep(1e4);
  }
  mpc.printTiming();
}

TEST(MPC, TakeOffTest) {
  Quadrotor quad(M, L);
  std::shared_ptr<MpcParameters> params = std::make_shared<MpcParameters>();
  MpcController mpc(quad, params);

  QuadState initial_state;
  initial_state.setZero();
  QuadState start_state = initial_state;
  start_state.p.z() = 1.0;
  Command hover_command;
  hover_command.t = 0.0;
  hover_command.thrusts.setConstant(M * G / 4.0);

  SetpointVector references(20, Setpoint(start_state, hover_command));
  SetpointVector setpoints;

  QuadState state = initial_state;
  for (int i = 0; i < 20; ++i) {
    state.p.z() = i * 0.1;
    mpc.getCommand(state, references, &setpoints);
    Command command = setpoints.front().input;
    if (i < 10)
      EXPECT_GT(command.thrusts.sum(), M * G);
    else if (i == 10)
      EXPECT_NEAR(command.thrusts.sum(), M * G, 1e-3);
    else
      EXPECT_LT(command.thrusts.sum(), M * G);
    usleep(1e4);
  }
  mpc.printTiming();
}

TEST(MPC, TakeOffTestCollectiveThrustRates) {
  Quadrotor quad(M, L);
  std::shared_ptr<MpcParameters> params = std::make_shared<MpcParameters>();
  MpcController mpc(quad, params);

  QuadState initial_state;
  initial_state.setZero();
  QuadState start_state = initial_state;
  start_state.p.z() = 1.0;
  Command hover_command;
  hover_command.t = 0.0;
  hover_command.omega.setZero();
  hover_command.collective_thrust = G;

  SetpointVector references(20, Setpoint(start_state, hover_command));
  SetpointVector setpoints;

  QuadState state = initial_state;
  for (int i = 0; i < 20; ++i) {
    state.p.z() = i * 0.1;
    mpc.getCommand(state, references, &setpoints);
    Command command = setpoints.front().input;
    if (i < 10)
      EXPECT_GT(command.thrusts.sum(), M * G);
    else if (i == 10)
      EXPECT_NEAR(command.thrusts.sum(), M * G, 1e-3);
    else
      EXPECT_LT(command.thrusts.sum(), M * G);
    usleep(1e4);
  }
  mpc.printTiming();
}

TEST(MPC, StepReferenceJump) {
  static constexpr Scalar dt = 0.02;
  static constexpr Scalar T = 8.0;
  Quadrotor quad(M, L);
  std::shared_ptr<MpcParameters> params = std::make_shared<MpcParameters>();
  MpcController mpc(quad, params);

  QuadrotorSimulator sim(quad);
  sim.addModel(ModelMotor{quad});
  sim.addModel(ModelThrustTorqueSimple{quad});
  sim.addModel(ModelRigidBody{quad});

  QuadState initial_state;
  initial_state.setZero();

  sim.reset(initial_state);

  QuadState reference_state;
  reference_state.t = 0.0;
  reference_state.setZero();
  reference_state.p = Vector<3>(1.0, 1.0, 1.0);
  reference_state.q(0.5 * M_PI);

  Command reference_command;
  reference_command.t = 0.0;
  reference_command.thrusts.setConstant(9.8066 * M / 4.0);

  Setpoint reference;
  reference.state = reference_state;
  reference.input = reference_command;
  SetpointVector references({reference});
  SetpointVector setpoints;
  QuadState state;
  Command command;
  Timer timer("MpcSim");
  for (Scalar t = 0.0; t <= T; t += dt) {
    timer.tic();
    EXPECT_TRUE(sim.getState(&state));
    mpc.getCommand(state, references, &setpoints);
    Command command = setpoints.front().input;
    sim.setCommand(command);
    sim.run(dt);
    timer.toc();
    usleep(1e6 * std::max(0.0, dt - timer.last()));
  }

  std::cout << timer;

  // copy motors to ignore them
  state.mot = reference_state.mot;
  state.motdes = reference_state.motdes;

  EXPECT_TRUE(state.x.isApprox(reference_state.x, 0.3))
    << "Actual:   " << state << std::endl
    << "Expected: " << reference_state << std::endl;
}
