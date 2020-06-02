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
    
    self._p = None
    self._time_step = 1./240.
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

    
    ctrl_size = 43  #numDof
    root_size = 7 # root
    
    action_dim = ctrl_size - root_size
    
    action_bound_min = np.array([
        -4.79999999999, -1.00000000000, -1.00000000000, -1.00000000000, -4.00000000000,
        -1.00000000000, -1.00000000000, -1.00000000000, -7.77999999999, -1.00000000000,
        -1.000000000, -1.000000000, -7.850000000, -6.280000000, -1.000000000, -1.000000000,
        -1.000000000, -12.56000000, -1.000000000, -1.000000000, -1.000000000, -4.710000000,
        -7.779999999, -1.000000000, -1.000000000, -1.000000000, -7.850000000, -6.280000000,
        -1.000000000, -1.000000000, -1.000000000, -8.460000000, -1.000000000, -1.000000000,
        -1.000000000, -4.710000000
    ])
    
    #print("len(action_bound_min)=",len(action_bound_min))
    action_bound_max = np.array([
        4.799999999, 1.000000000, 1.000000000, 1.000000000, 4.000000000, 1.000000000, 1.000000000,
        1.000000000, 8.779999999, 1.000000000, 1.0000000, 1.0000000, 4.7100000, 6.2800000,
        1.0000000, 1.0000000, 1.0000000, 12.560000, 1.0000000, 1.0000000, 1.0000000, 7.8500000,
        8.7799999, 1.0000000, 1.0000000, 1.0000000, 4.7100000, 6.2800000, 1.0000000, 1.0000000,
        1.0000000, 10.100000, 1.0000000, 1.0000000, 1.0000000, 7.8500000
    ])
    #print("len(action_bound_max)=",len(action_bound_max))
    
    self.action_space = spaces.Box(action_bound_min, action_bound_max)
    observation_min = np.array([0.0]+[-100.0]+[-4.0]*105+[-500.0]*90)
    observation_max = np.array([1.0]+[100.0]+[4.0]*105+[500.0]*90)
    state_size = 197
    self.observation_space = spaces.Box(observation_min, observation_min, dtype=np.float32)

    self.seed()
    
    self.viewer = None
    self._configure()
    
    # Query the policy at 30Hz
    self.policy_query_30 = True

  def _configure(self, display=None):
    self.display = display

  def seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]

  def step(self, action):
    
    #apply control action
    agent_id = -1
    self._internal_env.set_action(agent_id, action)
    
    #step sim
    self._internal_env.update(self._time_step)
    
    if self.policy_query_30:
      # Step simulation until a new action is required
      while not self._internal_env.need_new_action(agent_id):
        self._p.stepSimulation()
        self._internal_env.t += self._time_step
          # print("t:", self._internal_env.t)
    # print("next update time:", self._internal_env.needs_update_time)
    # print("Performing update")
        
    #record state
    self.state = self._internal_env.record_state(agent_id)
    
    #record reward
    reward = self._internal_env.calc_reward(agent_id)
    
    #record done
    done = self._internal_env.is_episode_end()
    
    return np.array(self.state), reward, done, {}

  def reset(self):
    #    print("-----------reset simulation---------------")
    if self._internal_env==None:
      self._internal_env = PyBulletDeepMimicEnv(self._arg_parser, self._renders)
    self._internal_env.reset()
    self._p = self._internal_env._pybullet_client
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
    if (not self._p == None):
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
    
    pass

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
