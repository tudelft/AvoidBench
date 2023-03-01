#!/usr/bin/env python3
import argparse
import math
#
import os

import numpy as np
import torch
from flightgym import AvoidVisionEnv_v1
from ruamel.yaml import YAML, RoundTripDumper, dump
from stable_baselines3.common.utils import get_device
from stable_baselines3.ppo.policies import MlpPolicy

from rpg_baselines.torch.common.ppo import PPO
from rpg_baselines.torch.envs import vec_vision_env_wrapper as wrapper
from rpg_baselines.torch.common.util import test_policy
