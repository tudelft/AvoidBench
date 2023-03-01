#!/usr/bin/env python3
import argparse
import sys
import math
#ros dependency
import rospy
import rospkg
from nav_msgs.msg import Path, Odometry
from rpg_quadrotor_msgs.msg import ControlCommand
from mav_msgs.msg import Actuators
#
import os

import numpy as np
import torch
from flightgym import TrackingEnv_v1
from ruamel.yaml import YAML, RoundTripDumper, dump
from stable_baselines3.common.utils import get_device
from stable_baselines3.ppo.policies import MlpPolicy

from rpg_baselines.torch.common.ppo import PPO
from rpg_baselines.torch.envs import vec_env_wrapper as wrapper

class RLController():
    def __init__(self, env, model):
        self.env = env
        self.model = model
        self.reset_state = False
        self.state = None
        self.motor_speed = None
        self.state_est_sub_ = rospy.Subscriber("/iris/ground_truth/odometry", Odometry, self.poseCallback,
                                                queue_size=1, tcp_nodelay=True)
        self.motors_sub_ = rospy.Subscriber("/iris/motor_speed", Actuators, self.motorCallback,
                                                queue_size=1, tcp_nodelay=True)
        self.control_command_pub_ = rospy.Publisher("/iris/autopilot/control_command_input", ControlCommand, queue_size=1)
        self.odometry_test_pub_ = rospy.Publisher("/rl_test", Odometry, queue_size=10)
        self.pub_cmd_loop_ = rospy.Timer(rospy.Duration(0.01), self.pubCMD)
        # self.target_point_sub_ = rospy.Subscriber("/hummingbird/goal_point", Path, self.TargetCallback, queue_size=1)
        self.obs = self.env.reset()
        self.done = False
        self.ep_len = 0

    def poseCallback(self, data):
        self.state = np.zeros(14, dtype=np.float64)
        self.state[10] = -0.8 - data.pose.pose.position.x
        self.state[11] = 0.8 - data.pose.pose.position.y
        self.state[12] = 1.9 - data.pose.pose.position.z
        self.state[13] = 0.0
        self.state[0] = data.twist.twist.linear.x
        self.state[1] = data.twist.twist.linear.y
        self.state[2] = data.twist.twist.linear.z
        self.state[3] = data.twist.twist.angular.x
        self.state[4] = data.twist.twist.angular.y
        self.state[5] = data.twist.twist.angular.z
        self.state[6] = data.pose.pose.orientation.w
        self.state[7] = data.pose.pose.orientation.x
        self.state[8] = data.pose.pose.orientation.y
        self.state[9] = data.pose.pose.orientation.z

    def motorCallback(self, data):
        self.motor_speed = np.zeros(4, dtype=np.float64)
        self.motor_speed[0] = data.angular_velocities[0]
        self.motor_speed[1] = data.angular_velocities[1]
        self.motor_speed[2] = -data.angular_velocities[2]
        self.motor_speed[3] = -data.angular_velocities[3]

    def pubCMD(self, data):
        if self.state is None:
            return
        if self.motor_speed is None:
            return
        self.env.setState(self.state)
        self.env.setMotor(self.motor_speed)
        # print("real: ", self.state[2])
        obs = self.env.resetFromAB()
        # print(obs)
        self.reset_state = True
        if self.ep_len < self.env.max_episode_steps and not self.done:
            act, _ = self.model.predict(obs, deterministic=True)
            act = np.array(act, dtype=np.float64)
            # print(act)
            test_obs = self.env.teststep(act)
            # print("predict: ", test_obs[0,2])
            self.runControl(act)
    
    def runControl(self, act):
        time_now = rospy.Time.now()
        mass = 1.05
        motor_omega_max_ = 1200.0
        thrust_map_ = np.array([8.54858e-06, 0.0, 0.0])
        thrust_max_ = (motor_omega_max_ * motor_omega_max_ * thrust_map_[0] +
                  motor_omega_max_ * thrust_map_[1] + thrust_map_[2]) * 4
        act_mean_ = np.array([thrust_max_/mass/2, 0.0, 0.0, 0.0])
        act_std_  = np.array([thrust_max_/mass/2, 4.0, 4.0, 1.5])
        pi_act_ = act * act_std_ + act_mean_
        # print(pi_act_)
        cmd = ControlCommand()
        cmd.control_mode = ControlCommand.BODY_RATES
        cmd.armed = True
        cmd.header.stamp = time_now
        cmd.bodyrates.x = pi_act_[1]
        cmd.bodyrates.y = pi_act_[2]
        cmd.bodyrates.z = pi_act_[3]
        cmd.collective_thrust = pi_act_[0]
        cmd.expected_execution_time = time_now + rospy.Duration(0.01)
        self.control_command_pub_.publish(cmd)

def parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=0, help="Random seed")
    parser.add_argument("--train", type=int, default=0, help="Train the policy or evaluate the policy")
    parser.add_argument("--render", type=int, default=0, help="Render with Unity")
    parser.add_argument("--trial", type=int, default=1, help="PPO trial number")
    parser.add_argument("--iter", type=int, default=100, help="PPO iter number")
    return parser


def main():
    try:
        rospy.get_master().getPid()
    except:
        print("roscore is offline, exit")
        sys.exit(-1)
    rospy.init_node('avoid_manage', anonymous=True)

    args = parser().parse_args()
    cfg = YAML().load(
        open(
            os.environ["AVOIDBENCH_PATH"] + "/flightpy/configs/control/config_gazebo.yaml", "r"
        )
    )
    cfg["simulation"]["num_envs"] = 1
    eval_env = wrapper.FlightEnvVec(
        TrackingEnv_v1(dump(cfg, Dumper=RoundTripDumper), False)
    )
    weight = os.environ["AVOIDBENCH_PATH"] + "/flightpy/flightrl/saved/PPO_{0}/Policy/iter_{1:05d}.pth".format(args.trial, args.iter)
    env_rms = os.environ["AVOIDBENCH_PATH"] +"/flightpy/flightrl/saved/PPO_{0}/RMS/iter_{1:05d}.npz".format(args.trial, args.iter)

    device = get_device("auto")
    saved_variables = torch.load(weight, map_location=device)
    # Create policy object
    policy = MlpPolicy(**saved_variables["data"])
    #
    policy.action_net = torch.nn.Sequential(policy.action_net, torch.nn.Tanh())
    # Load weights
    policy.load_state_dict(saved_variables["state_dict"], strict=False)
    policy.to(device)

    eval_env.load_rms(env_rms)

    controller = RLController(eval_env, policy)
    rospy.spin()


if __name__ == "__main__":
    main()
