"""
Classic cart-pole system implemented by Rich Sutton et al.
Copied from https://webdocs.cs.ualberta.ca/~sutton/book/code/pole.c
"""
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import logging
import math
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import time
import subprocess
import pybullet as p2
import pybullet_data
import pybullet_utils.bullet_client as bc
from pkg_resources import parse_version
from pybullet_envs.deep_mimic.env.pybullet_deep_mimic_env import PyBulletDeepMimicEnv
from pybullet_utils.arg_parser import ArgParser
from pybullet_utils.logger import Logger

logger = logging.getLogger(__name__)


class HumanoidDeepBulletEnv(gym.Env):
  metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}

  def __init__(self, renders=False, arg_file=''):
    
    self._arg_parser = ArgParser()
    Logger.print2("===========================================================")
    succ = False
    if (arg_file != ''):
      path = pybullet_data.getDataPath() + "/args/" + arg_file
      succ = self._arg_parser.load_file(path)
      Logger.print2(arg_file)
    assert succ, Logger.print2('Failed to load args from: ' + arg_file)
    
    self._internal_env =  None
    self._renders = renders
    self._discrete_actions = False
    self._arg_file=arg_file
    self._render_height = 200
    self._render_width = 320
    
    self.theta_threshold_radians = 12 * 2 * math.pi / 360
    self.x_threshold = 0.4  #2.4
    high = np.array([
        self.x_threshold * 2,
        np.finfo(np.float32).max, self.theta_threshold_radians * 2,
        np.finfo(np.float32).max
    ])

    self.force_mag = 10

    if self._discrete_actions:
      self.action_space = spaces.Discrete(2)
    else:
      action_dim = 1
      action_high = np.array([self.force_mag] * action_dim)
      self.action_space = spaces.Box(-action_high, action_high)
    
    self.observation_space = spaces.Box(-high, high, dtype=np.float32)

    self.seed()
    
    self.viewer = None
    self._configure()

  def _configure(self, display=None):
    self.display = display

  def seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]

  def step(self, action):
    p = self._p
    if self._discrete_actions:
      force = self.force_mag if action == 1 else -self.force_mag
    else:
      force = action[0]

    p.setJointMotorControl2(self.cartpole, 0, p.TORQUE_CONTROL, force=force)
    p.stepSimulation()

    self.state = p.getJointState(self.cartpole, 1)[0:2] + p.getJointState(self.cartpole, 0)[0:2]
    theta, theta_dot, x, x_dot = self.state

    done =  x < -self.x_threshold \
                or x > self.x_threshold \
                or theta < -self.theta_threshold_radians \
                or theta > self.theta_threshold_radians
    done = bool(done)
    reward = 1.0
    #print("state=",self.state)
    return np.array(self.state), reward, done, {}

  def reset(self):
    #    print("-----------reset simulation---------------")
    if self._internal_env==None:
      self._internal_env = PyBulletDeepMimicEnv(self._arg_parser, self._renders)
    self._internal_env.reset()
    agent_id = -1 #unused here
    state = self._internal_env.record_state(agent_id)
    return state

  def render(self, mode='human', close=False):
    if mode == "human":
      self._renders = True
    if mode != "rgb_array":
      return np.array([])
    base_pos=[0,0,0]
    self._cam_dist = 2
    self._cam_pitch = 0.3
    self._cam_yaw = 0
    if (self._physics_client_id>=0):
      view_matrix = self._p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=base_pos,
        distance=self._cam_dist,
        yaw=self._cam_yaw,
        pitch=self._cam_pitch,
        roll=0,
        upAxisIndex=2)
      proj_matrix = self._p.computeProjectionMatrixFOV(fov=60,
             aspect=float(self._render_width) /
             self._render_height,
             nearVal=0.1,
             farVal=100.0)
      (_, _, px, _, _) = self._p.getCameraImage(
          width=self._render_width,
          height=self._render_height,
          renderer=self._p.ER_BULLET_HARDWARE_OPENGL,
          viewMatrix=view_matrix,
          projectionMatrix=proj_matrix)
    else:
      px = np.array([[[255,255,255,255]]*self._render_width]*self._render_height, dtype=np.uint8)
    rgb_array = np.array(px, dtype=np.uint8)
    rgb_array = np.reshape(np.array(px), (self._render_height, self._render_width, -1))
    rgb_array = rgb_array[:, :, :3]
    return rgb_array

  def configure(self, args):
    pass
    
  def close(self):
    if self._physics_client_id >= 0:
      self._p.disconnect()
    self._physics_client_id = -1

class HumanoidDeepMimicBackflipBulletEnv(HumanoidDeepBulletEnv):
  metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}

  def __init__(self, renders=False):
    # start the bullet physics server
    HumanoidDeepBulletEnv.__init__(self, renders, arg_file="run_humanoid3d_backflip_args.txt")


class HumanoidDeepMimicWalkBulletEnv(HumanoidDeepBulletEnv):
  metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}

  def __init__(self, renders=False):
    # start the bullet physics server
    HumanoidDeepBulletEnv.__init__(self, renders, arg_file="run_humanoid3d_walk_args.txt")

class CartPoleContinuousBulletEnv5(HumanoidDeepBulletEnv):
  metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}

  def __init__(self, renders=False):
    # start the bullet physics server
    HumanoidDeepBulletEnv.__init__(self, renders, arg_file="")
