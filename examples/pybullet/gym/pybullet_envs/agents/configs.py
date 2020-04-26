# Copyright 2017 The TensorFlow Agents Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Example configurations using the PPO algorithm."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import functools

from . import ppo
from . import networks
from pybullet_envs.bullet import minitaur_gym_env
from pybullet_envs.bullet import minitaur_duck_gym_env
from pybullet_envs.bullet import minitaur_env_randomizer
import pybullet_envs.bullet.minitaur_gym_env as minitaur_gym_env
import pybullet_envs
try:
  import tensorflow.compat.v1 as tf
except Exception:
  import tensorflow as tf


def default():
  """Default configuration for PPO."""
  # General
  algorithm = ppo.PPOAlgorithm
  num_agents = 30
  eval_episodes = 30
  use_gpu = False
  # Network
  network = networks.feed_forward_gaussian
  weight_summaries = dict(all=r'.*', policy=r'.*/policy/.*', value=r'.*/value/.*')
  policy_layers = 200, 100
  value_layers = 200, 100
  init_mean_factor = 0.1
  init_logstd = -1
  # Optimization
  update_every = 30
  update_epochs = 25
  optimizer = tf.train.AdamOptimizer
  update_epochs_policy = 64
  update_epochs_value = 64
  learning_rate = 1e-4
  # Losses
  discount = 0.995
  kl_target = 1e-2
  kl_cutoff_factor = 2
  kl_cutoff_coef = 1000
  kl_init_penalty = 1
  return locals()


def pybullet_pendulum():
  locals().update(default())
  env = 'InvertedPendulumBulletEnv-v0'
  max_length = 200
  steps = 5e7  # 50M
  return locals()


def pybullet_doublependulum():
  locals().update(default())
  env = 'InvertedDoublePendulumBulletEnv-v0'
  max_length = 1000
  steps = 5e7  # 50M
  return locals()


def pybullet_pendulumswingup():
  locals().update(default())
  env = 'InvertedPendulumSwingupBulletEnv-v0'
  max_length = 1000
  steps = 5e7  # 50M
  return locals()


def pybullet_cheetah():
  """Configuration for MuJoCo's half cheetah task."""
  locals().update(default())
  # Environment
  env = 'HalfCheetahBulletEnv-v0'
  max_length = 1000
  steps = 1e8  # 100M
  return locals()


def pybullet_ant():
  locals().update(default())
  env = 'AntBulletEnv-v0'
  max_length = 1000
  steps = 5e7  # 50M
  return locals()


def pybullet_kuka_grasping():
  """Configuration for Bullet Kuka grasping task."""
  locals().update(default())
  # Environment
  env = 'KukaBulletEnv-v0'
  max_length = 1000
  steps = 1e7  # 10M
  return locals()


def pybullet_racecar():
  """Configuration for Bullet MIT Racecar task."""
  locals().update(default())
  # Environment
  env = 'RacecarBulletEnv-v0'  #functools.partial(racecarGymEnv.RacecarGymEnv, isDiscrete=False, renders=True)
  max_length = 10
  steps = 1e7  # 10M
  return locals()


def pybullet_humanoid():
  locals().update(default())
  randomizer = (minitaur_env_randomizer.MinitaurEnvRandomizer())
  env = 'HumanoidBulletEnv-v0'
  max_length = 1000
  steps = 3e8  # 300M
  return locals()


def pybullet_minitaur():
  """Configuration specific to minitaur_gym_env.MinitaurBulletEnv class."""
  locals().update(default())
  randomizer = (minitaur_env_randomizer.MinitaurEnvRandomizer())
  env = functools.partial(minitaur_gym_env.MinitaurBulletEnv,
                          accurate_motor_model_enabled=True,
                          motor_overheat_protection=True,
                          pd_control_enabled=True,
                          env_randomizer=randomizer,
                          render=False)
  max_length = 1000
  steps = 3e7  # 30M
  return locals()


def pybullet_duck_minitaur():
  """Configuration specific to minitaur_gym_env.MinitaurBulletDuckEnv class."""
  locals().update(default())
  randomizer = (minitaur_env_randomizer.MinitaurEnvRandomizer())
  env = functools.partial(minitaur_gym_env.MinitaurBulletDuckEnv,
                          accurate_motor_model_enabled=True,
                          motor_overheat_protection=True,
                          pd_control_enabled=True,
                          env_randomizer=randomizer,
                          render=False)
  max_length = 1000
  steps = 3e7  # 30M
  return locals()
