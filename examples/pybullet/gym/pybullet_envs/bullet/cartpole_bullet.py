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

    observation_high = np.array([
          np.finfo(np.float32).max,
          np.finfo(np.float32).max,
          np.finfo(np.float32).max,
          np.finfo(np.float32).max])
    action_high = np.array([0.1])

    self.action_space = spaces.Discrete(9)
    self.observation_space = spaces.Box(-observation_high, observation_high)

    self.theta_threshold_radians = 1
    self.x_threshold = 2.4
    self._seed()
#    self.reset()
    self.viewer = None
    self._configure()

  def _configure(self, display=None):
    self.display = display

  def _seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]

  def _step(self, action):
    p.stepSimulation()
#    time.sleep(self.timeStep)
    self.state = p.getJointState(self.cartpole, 1)[0:2] + p.getJointState(self.cartpole, 0)[0:2]
    theta, theta_dot, x, x_dot = self.state
    
    dv = 0.1
    deltav = [-10.*dv,-5.*dv, -2.*dv, -0.1*dv, 0, 0.1*dv, 2.*dv,5.*dv, 10.*dv][action]
    
    p.setJointMotorControl2(self.cartpole, 0, p.VELOCITY_CONTROL, targetVelocity=(deltav + self.state[3]))

    done =  x < -self.x_threshold \
                or x > self.x_threshold \
                or theta < -self.theta_threshold_radians \
                or theta > self.theta_threshold_radians
    reward = 1.0

    return np.array(self.state), reward, done, {}

  def _reset(self):
#    print("-----------reset simulation---------------")
    p.resetSimulation()
    self.cartpole = p.loadURDF(os.path.join(pybullet_data.getDataPath(),"cartpole.urdf"),[0,0,0])
    self.timeStep = 0.01
    p.setJointMotorControl2(self.cartpole, 1, p.VELOCITY_CONTROL, force=0)
    p.setGravity(0,0, -10)
    p.setTimeStep(self.timeStep)
    p.setRealTimeSimulation(0)

    initialCartPos = self.np_random.uniform(low=-0.5, high=0.5, size=(1,))
    initialAngle = self.np_random.uniform(low=-0.5, high=0.5, size=(1,))
    p.resetJointState(self.cartpole, 1, initialAngle)
    p.resetJointState(self.cartpole, 0, initialCartPos)

    self.state = p.getJointState(self.cartpole, 1)[0:2] + p.getJointState(self.cartpole, 0)[0:2]

    return np.array(self.state)

  def _render(self, mode='human', close=False):
      return
