"""
Classic cart-pole system implemented by Rich Sutton et al.
Copied from https://webdocs.cs.ualberta.ca/~sutton/book/code/pole.c
"""
import os,  inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)

import logging
import math
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import time
import subprocess
import pybullet as p
import pybullet_data
from pkg_resources import parse_version

logger = logging.getLogger(__name__)

class CartPoleBulletEnv(gym.Env):
  metadata = {
    'render.modes': ['human', 'rgb_array'],
    'video.frames_per_second' : 50
  }

  def __init__(self, renders=True):
    # start the bullet physics server
    self._renders = renders
    if (renders):
	    p.connect(p.GUI)
    else:
    	p.connect(p.DIRECT)
    self.theta_threshold_radians = 12 * 2 * math.pi / 360
    self.x_threshold = 0.4 #2.4
    high = np.array([
         self.x_threshold * 2,
         np.finfo(np.float32).max,
         self.theta_threshold_radians * 2,
         np.finfo(np.float32).max])

    self.force_mag = 10

    self.action_space = spaces.Discrete(2)
    self.observation_space = spaces.Box(-high, high, dtype=np.float32)

    self.seed()
    #    self.reset()
    self.viewer = None
    self._configure()

  def _configure(self, display=None):
    self.display = display

  def seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]

  def step(self, action):
    force = self.force_mag if action==1 else -self.force_mag

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
    p.resetSimulation()
    self.cartpole = p.loadURDF(os.path.join(pybullet_data.getDataPath(),"cartpole.urdf"),[0,0,0])
    p.changeDynamics(self.cartpole, -1, linearDamping=0, angularDamping=0)
    p.changeDynamics(self.cartpole, 0, linearDamping=0, angularDamping=0)
    p.changeDynamics(self.cartpole, 1, linearDamping=0, angularDamping=0)
    self.timeStep = 0.02
    p.setJointMotorControl2(self.cartpole, 1, p.VELOCITY_CONTROL, force=0)
    p.setJointMotorControl2(self.cartpole, 0, p.VELOCITY_CONTROL, force=0)
    p.setGravity(0,0, -9.8)
    p.setTimeStep(self.timeStep)
    p.setRealTimeSimulation(0)

    randstate = self.np_random.uniform(low=-0.05, high=0.05, size=(4,))
    p.resetJointState(self.cartpole, 1, randstate[0], randstate[1])
    p.resetJointState(self.cartpole, 0, randstate[2], randstate[3])
    #print("randstate=",randstate)
    self.state = p.getJointState(self.cartpole, 1)[0:2] + p.getJointState(self.cartpole, 0)[0:2]
    #print("self.state=", self.state)
    return np.array(self.state)

  def render(self, mode='human', close=False):
      return
