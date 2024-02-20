#include "agilib/simulator/quadrotor_simulator.hpp"

#include <gtest/gtest.h>

#include "agilib/math/gravity.hpp"
#include "agilib/simulator/model_propeller_bem.hpp"
#include "agilib/types/quad_state.hpp"

using namespace agi;

static constexpr Scalar CTRL_UPDATE_FREQUENCY = 50.0;
static constexpr int SIM_STEPS_N = 20;

TEST(QuadrotorSimulator, Constructor) {
  QuadrotorSimulator quad_sim;

  QuadState quad_state;
  quad_sim.getState(&quad_state);

  QuadState expected_state;
  expected_state.setZero();
  expected_state.x(QS::ATTW) = 1.0;

  EXPECT_EQ(expected_state.x(QS::ATTW), quad_state.x(QS::ATTW));
  EXPECT_TRUE(quad_state.x.isApprox(expected_state.x));
}

TEST(QuadrotorSimulator, ResetSimulator) {
  QuadrotorSimulator quad_sim;
  QuadState initial_state;
  QuadState quad_state;

  // default reset
  initial_state.setZero();

  EXPECT_TRUE(quad_sim.reset());
  EXPECT_TRUE(quad_sim.getState(&quad_state));

  EXPECT_TRUE(quad_state.x.isApprox(initial_state.x));
  EXPECT_EQ(quad_state.t, 0.0);

  // randomly reset the quadrotor state
  initial_state.setZero();
  initial_state.x = Vector<QuadState::SIZE>::Random();

  EXPECT_TRUE(quad_sim.reset(initial_state));
  EXPECT_TRUE(quad_sim.getState(&quad_state));

  EXPECT_TRUE(quad_state.x.isApprox(initial_state.x));
  EXPECT_EQ(quad_state.t, 0.0);

  // check failure case
  QuadState initial_state_nan;
  EXPECT_FALSE(quad_sim.reset(initial_state_nan));
  EXPECT_FALSE(quad_sim.setState(initial_state_nan));
}

TEST(QuadrotorSimulator, BEM) {
  QuadrotorSimulator quad_sim;
  Quadrotor quad = quad_sim.getQuadrotor();
  quad.motor_tau_inv_ = NAN;

  quad_sim.clearModelPipeline();
  quad_sim.addModel(ModelInit{quad});
  quad_sim.addModel(ModelPropellerBEM{quad});
  quad_sim.addModel(ModelRigidBody{quad});
  EXPECT_TRUE(quad_sim.updateQuad(quad));

  const Scalar ctl_dt = (1.0 / CTRL_UPDATE_FREQUENCY);

  QuadState quad_state;
  QuadState final_state;

  // hovering test
  quad_state.setZero();
  quad_state.p << 0.0, 0.0, 1.0;
  quad_sim.reset(quad_state);

  Command cmd;
  cmd.t = 0.0;
  // 0.9575 due to frame obscurring parts of the area below
  cmd.thrusts = Vector<4>::Constant(G * quad.m_) / 4.0 / 0.9575;


  for (int i = 0; i < SIM_STEPS_N; i++) {
    // run quadrotor simulator
    EXPECT_TRUE(quad_sim.setCommand(cmd));
    EXPECT_TRUE(quad_sim.run(ctl_dt));
    quad_state.t += ctl_dt;
  }
  quad_sim.getState(&final_state);

  // copy motors to ignore them
  final_state.mot = quad_state.mot;
  final_state.motdes = quad_state.motdes;

  EXPECT_NEAR(final_state.t, quad_state.t, 1e-9);
  EXPECT_TRUE((quad_state.p - final_state.p).isMuchSmallerThan(1, 2e-2))
    << "Actual:   " << final_state << std::endl
    << "Expected: " << quad_state << std::endl;
  EXPECT_TRUE((quad_state.v - final_state.v).isMuchSmallerThan(1, 1e-1))
    << "Actual:   " << final_state << std::endl
    << "Expected: " << quad_state << std::endl;
  EXPECT_TRUE((quad_state.a - final_state.a).isMuchSmallerThan(1, 2e-1))
    << "Actual:   " << final_state << std::endl
    << "Expected: " << quad_state << std::endl;
  EXPECT_TRUE((quad_state.qx - final_state.qx).isMuchSmallerThan(1, 1e-6))
    << "Actual:   " << final_state << std::endl
    << "Expected: " << quad_state << std::endl;
  EXPECT_TRUE((quad_state.w - final_state.w).isMuchSmallerThan(1, 1e-6))
    << "Actual:   " << final_state << std::endl
    << "Expected: " << quad_state << std::endl;

  // free fall
  quad_state.setZero();
  quad_state.p << 0.0, 0.0, 1.0;
  quad_sim.reset(quad_state);

  cmd.t = 0.0;
  cmd.thrusts << 0.0, 0.0, 0.0, 0.0;

  for (int i = 0; i < SIM_STEPS_N; i++) {
    // run quadrotor simulator
    quad_sim.setCommand(cmd);
    quad_sim.run(ctl_dt);

    // manually update the state
    quad_state.p += quad_state.v * ctl_dt + ctl_dt * ctl_dt / 2.0 * GVEC;
    quad_state.v += ctl_dt * GVEC;
    quad_state.a = GVEC;
    quad_state.t += ctl_dt;
  }

  final_state.setZero();
  quad_sim.getState(&final_state);

  // copy motors to ignore them
  final_state.mot = quad_state.mot;
  final_state.motdes = quad_state.motdes;

  EXPECT_NEAR(final_state.t, quad_state.t, 1e-9);
  EXPECT_NEAR(final_state.t, quad_state.t, 1e-9);
  EXPECT_TRUE((quad_state.p - final_state.p).isMuchSmallerThan(1, 2e-2))
    << "Actual:   " << final_state << std::endl
    << "Expected: " << quad_state << std::endl;
  EXPECT_TRUE((quad_state.v - final_state.v).isMuchSmallerThan(1, 1e-1))
    << "Actual:   " << final_state << std::endl
    << "Expected: " << quad_state << std::endl;
  EXPECT_TRUE((quad_state.a - final_state.a).isMuchSmallerThan(1, 5e-1))
    << "Actual:   " << final_state << std::endl
    << "Expected: " << quad_state << std::endl;
  EXPECT_TRUE((quad_state.qx - final_state.qx).isMuchSmallerThan(1, 1e-6))
    << "Actual:   " << final_state << std::endl
    << "Expected: " << quad_state << std::endl;
  EXPECT_TRUE((quad_state.w - final_state.w).isMuchSmallerThan(1, 1e-6))
    << "Actual:   " << final_state << std::endl
    << "Expected: " << quad_state << std::endl;
}

TEST(QuadrotorSimulator, RunSimulatorCmdFeedthrough) {
  QuadrotorSimulator quad_sim;
  Quadrotor quad = quad_sim.getQuadrotor();
  quad.motor_tau_inv_ = NAN;
  quad_sim.clearModelPipeline();
  quad_sim.addModel(ModelInit{quad});
  quad_sim.addModel(ModelThrustTorqueSimple{quad});
  quad_sim.addModel(ModelRigidBody{quad});
  EXPECT_TRUE(quad_sim.updateQuad(quad));

  const Scalar ctl_dt = (1.0 / CTRL_UPDATE_FREQUENCY);

  QuadState quad_state;
  QuadState final_state;

  // hovering test
  quad_state.setZero();
  quad_state.p << 0.0, 0.0, 1.0;
  quad_sim.reset(quad_state);

  Command cmd;
  cmd.t = 0.0;
  cmd.thrusts = Vector<4>::Constant(G * quad.m_) / 4.0;


  for (int i = 0; i < SIM_STEPS_N; i++) {
    // run quadrotor simulator
    EXPECT_TRUE(quad_sim.setCommand(cmd));
    EXPECT_TRUE(quad_sim.run(ctl_dt));
    quad_state.t += ctl_dt;
  }
  quad_sim.getState(&final_state);

  // copy motors to ignore them
  final_state.mot = quad_state.mot;
  final_state.motdes = quad_state.motdes;

  EXPECT_NEAR(final_state.t, quad_state.t, 1e-9);
  EXPECT_TRUE(quad_state.x.isApprox(final_state.x))
    << "Actual:   " << final_state << std::endl
    << "Expected: " << quad_state << std::endl;

  // free fall
  quad_state.setZero();
  quad_state.p << 0.0, 0.0, 1.0;
  quad_sim.reset(quad_state);

  cmd.t = 0.0;
  cmd.thrusts << 0.0, 0.0, 0.0, 0.0;

  for (int i = 0; i < SIM_STEPS_N; i++) {
    // run quadrotor simulator
    quad_sim.setCommand(cmd);
    quad_sim.run(ctl_dt);

    // manually update the state
    quad_state.p += quad_state.v * ctl_dt + ctl_dt * ctl_dt / 2.0 * GVEC;
    quad_state.v += ctl_dt * GVEC;
    quad_state.a = GVEC;
    quad_state.t += ctl_dt;
  }

  final_state.setZero();
  quad_sim.getState(&final_state);

  // copy motors to ignore them
  final_state.mot = quad_state.mot;
  final_state.motdes = quad_state.motdes;

  EXPECT_NEAR(final_state.t, quad_state.t, 1e-9);
  EXPECT_TRUE(quad_state.x.isApprox(final_state.x, 1e-3))
    << "Actual:   " << final_state << std::endl
    << "Expected: " << quad_state << std::endl;
}

TEST(QuadrotorSimulator, RunSimulatorBodyRate) {
  QuadrotorSimulator quad_sim;
  Quadrotor quad = quad_sim.getQuadrotor();
  quad.motor_tau_inv_ = NAN;
  quad_sim.clearModelPipeline();
  quad_sim.addModel(ModelInit{quad});
  quad_sim.addModel(ModelMotor{quad});
  quad_sim.addModel(ModelThrustTorqueSimple{quad});
  quad_sim.addModel(ModelRigidBody{quad});
  EXPECT_TRUE(quad_sim.updateQuad(quad));

  const Scalar ctl_dt = (1.0 / CTRL_UPDATE_FREQUENCY);

  QuadState quad_state;
  QuadState final_state;

  // hovering test
  quad_state.setZero();
  quad_state.p << 0.0, 0.0, 1.0;
  quad_sim.reset(quad_state);

  Command cmd;
  cmd.t = 0.0;
  cmd.collective_thrust = G;
  cmd.omega = Vector<3>::Zero();

  for (int i = 0; i < SIM_STEPS_N; i++) {
    // run quadrotor simulator
    EXPECT_TRUE(quad_sim.setCommand(cmd));
    EXPECT_TRUE(quad_sim.run(ctl_dt));

    quad_state.t += ctl_dt;
  }
  quad_sim.getState(&final_state);

  // copy motors to ignore them
  final_state.mot = quad_state.mot;
  final_state.motdes = quad_state.motdes;

  EXPECT_EQ(final_state.t, quad_state.t);
  EXPECT_TRUE(quad_state.x.isApprox(final_state.x, 1e-3))
    << "Actual:   " << final_state << std::endl
    << "Expected: " << quad_state << std::endl;

  // free fall
  quad_state.setZero();
  quad_state.p << 0.0, 0.0, 1.0;
  quad_sim.reset(quad_state);

  cmd.t = 0.0;
  cmd.collective_thrust = 0.0;
  cmd.omega << 0.0, 0.0, 0.0;

  for (int i = 0; i < SIM_STEPS_N; i++) {
    // run quadrotor simulator
    quad_sim.setCommand(cmd);
    quad_sim.run(ctl_dt);

    // manually update the state
    quad_state.p += quad_state.v * ctl_dt + ctl_dt * ctl_dt / 2.0 * GVEC;
    quad_state.v += ctl_dt * GVEC;
    quad_state.a = GVEC;
    quad_state.t += ctl_dt;
  }

  final_state.setZero();
  quad_sim.getState(&final_state);

  // copy motors to ignore them
  final_state.mot = quad_state.mot;
  final_state.motdes = quad_state.motdes;

  EXPECT_NEAR(final_state.t, quad_state.t, 1e-9);
  EXPECT_TRUE(quad_state.x.isApprox(final_state.x, 1e-3))
    << "Actual:   " << final_state << std::endl
    << "Expected: " << quad_state << std::endl;
}
