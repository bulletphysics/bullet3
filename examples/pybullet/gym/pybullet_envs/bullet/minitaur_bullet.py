import math
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import time

import pybullet as p
from pybullet_envs.bullet.minitaur import Minitaur

class MinitaurBulletEnv(gym.Env):
  metadata = {
      'render.modes': ['human', 'rgb_array'],
      'video.frames_per_second' : 50
  }

  def __init__(self):
    self._timeStep = 0.01
    self._observation = []
    self._envStepCounter = 0
    self._lastBasePosition = [0, 0, 0]

    p.connect(p.GUI)

    p.resetSimulation()
    p.setTimeStep(self._timeStep)
    p.loadURDF("plane.urdf")
    p.setGravity(0,0,-10)
    self._minitaur = Minitaur()

    observationDim = self._minitaur.getObservationDimension()
    observation_high = np.array([np.finfo(np.float32).max] * observationDim)
    actionDim = 8
    action_high = np.array([math.pi / 2.0] * actionDim)
    self.action_space = spaces.Box(-action_high, action_high)
    self.observation_space = spaces.Box(-observation_high, observation_high)

    self._seed()
    self.reset()
    self.viewer = None

  def __del__(self):
    p.disconnect()

  def _seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]

  def _step(self, action):
    if len(action) != self._minitaur.getActionDimension():
      raise ValueError("We expect {} continuous action not {}.".format(self._minitaur.getActionDimension(), len(actions.continuous_actions)))

    for i in range(len(action)):
      if not -math.pi/2 <= action[i] <= math.pi/2:
        raise ValueError("{}th action should be between -1 and 1 not {}.".format(i, action[i]))
      action[i] += math.pi / 2

    self._minitaur.applyAction(action)
    p.stepSimulation()
    self._observation = self._minitaur.getObservation()
    self._envStepCounter += 1
    reward = self._reward()
    done = self._termination()
    return np.array(self._observation), reward, done, {}

  def _reset(self):
    p.resetSimulation()
    p.setTimeStep(self._timeStep)
    p.loadURDF("plane.urdf")
    p.setGravity(0,0,-10)
    self._minitaur = Minitaur()
    self._observation = self._minitaur.getObservation()

  def _render(self, mode='human', close=False):
      return

  def is_fallen(self):
    orientation = self._minitaur.getBaseOrientation()
    rotMat = p.getMatrixFromQuaternion(orientation)
    localUp = rotMat[6:]
    return np.dot(np.asarray([0, 0, 1]), np.asarray(localUp)) < 0

  def _termination(self):
    return self.is_fallen()

  def _reward(self):
    currentBasePosition = self._minitaur.getBasePosition()
    reward = np.dot(np.asarray([-1, 0, 0]), np.asarray(currentBasePosition) - np.asarray(self._lastBasePosition))
    self._lastBasePosition = currentBasePosition
    return reward
