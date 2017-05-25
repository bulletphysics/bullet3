import math
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import time
import pybullet as p
import minitaur_new

class MinitaurGymEnv(gym.Env):
  metadata = {
      'render.modes': ['human', 'rgb_array'],
      'video.frames_per_second' : 50
  }

  def __init__(self,
               urdfRoot="",
               actionRepeat=1,
               isEnableSelfCollision=True,
               motorVelocityLimit=10.0,
               render=False):
    self._timeStep = 0.01
    self._urdfRoot = urdfRoot
    self._actionRepeat = actionRepeat
    self._motorVelocityLimit = motorVelocityLimit
    self._isEnableSelfCollision = isEnableSelfCollision
    self._observation = []
    self._envStepCounter = 0
    self._render = render
    self._lastBasePosition = [0, 0, 0]
    if self._render:
      p.connect(p.GUI)
    else:
      p.connect(p.DIRECT)
    self._seed()
    self.reset()
    observationDim = self._minitaur.getObservationDimension()
    observation_high = np.array([np.finfo(np.float32).max] * observationDim)
    actionDim = 8
    action_high = np.array([1] * actionDim)
    self.action_space = spaces.Box(-action_high, action_high)
    self.observation_space = spaces.Box(-observation_high, observation_high)
    self.viewer = None

  def _reset(self):
    p.resetSimulation()
    p.setPhysicsEngineParameter(numSolverIterations=300)
    p.setTimeStep(self._timeStep)
    p.loadURDF("%splane.urdf" % self._urdfRoot)
    p.setGravity(0,0,-10)
    self._minitaur = minitaur_new.Minitaur(urdfRootPath=self._urdfRoot, timeStep=self._timeStep, isEnableSelfCollision=self._isEnableSelfCollision, motorVelocityLimit=self._motorVelocityLimit)
    self._envStepCounter = 0
    self._lastBasePosition = [0, 0, 0]
    for i in range(100):
      p.stepSimulation()
    self._observation = self._minitaur.getObservation()
    return self._observation

  def __del__(self):
    p.disconnect()

  def _seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]

  def _step(self, action):
    if (self._render):
      basePos = self._minitaur.getBasePosition()
      p.resetDebugVisualizerCamera(1, 30, 40, basePos)

    if len(action) != self._minitaur.getActionDimension():
      raise ValueError("We expect {} continuous action not {}.".format(self._minitaur.getActionDimension(), len(action)))

    for i in range(len(action)):
      if not -1.01 <= action[i] <= 1.01:
        raise ValueError("{}th action should be between -1 and 1 not {}.".format(i, action[i]))

    realAction = self._minitaur.convertFromLegModel(action)
    self._minitaur.applyAction(realAction)
    for i in range(self._actionRepeat):
      p.stepSimulation()
      if self._render:
        time.sleep(self._timeStep)
      self._observation = self._minitaur.getObservation()
      if self._termination():
        break
      self._envStepCounter += 1
    reward = self._reward()
    done = self._termination()
    return np.array(self._observation), reward, done, {}

  def _render(self, mode='human', close=False):
      return

  def is_fallen(self):
    orientation = self._minitaur.getBaseOrientation()
    rotMat = p.getMatrixFromQuaternion(orientation)
    localUp = rotMat[6:]
    return np.dot(np.asarray([0, 0, 1]), np.asarray(localUp)) < 0 or self._observation[-1] < 0.1

  def _termination(self):
    return self.is_fallen()

  def _reward(self):
    currentBasePosition = self._minitaur.getBasePosition()
    forward_reward = currentBasePosition[0] - self._lastBasePosition[0]
    self._lastBasePosition = currentBasePosition

    energyWeight = 0.001
    energy = np.abs(np.dot(self._minitaur.getMotorTorques(), self._minitaur.getMotorVelocities())) * self._timeStep
    energy_reward = energyWeight * energy
    reward = forward_reward - energy_reward
    return reward
