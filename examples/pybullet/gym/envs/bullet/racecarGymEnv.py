import math
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import time
import pybullet as p
from . import racecar

class RacecarGymEnv(gym.Env):
  metadata = {
      'render.modes': ['human', 'rgb_array'],
      'video.frames_per_second' : 50
  }

  def __init__(self,
               urdfRoot="",
               actionRepeat=1,
               isEnableSelfCollision=True,
               render=True):
    print("init")
    self._timeStep = 0.01
    self._urdfRoot = urdfRoot
    self._actionRepeat = actionRepeat
    self._isEnableSelfCollision = isEnableSelfCollision
    self._observation = []
    self._ballUniqueId = -1
    self._envStepCounter = 0
    self._render = render
    self._p = p
    if self._render:
      p.connect(p.GUI)
    else:
      p.connect(p.DIRECT)
    self._seed()
    self.reset()
    observationDim = self._racecar.getObservationDimension()
    observation_high = np.array([np.finfo(np.float32).max] * observationDim)
    actionDim = 8
    action_high = np.array([1] * actionDim)
    self.action_space = spaces.Box(-action_high, action_high)
    self.observation_space = spaces.Box(-observation_high, observation_high)
    self.viewer = None

  def _reset(self):
    p.resetSimulation()
    #p.setPhysicsEngineParameter(numSolverIterations=300)
    p.setTimeStep(self._timeStep)
    #p.loadURDF("%splane.urdf" % self._urdfRoot)
    p.loadSDF("%sstadium.sdf" % self._urdfRoot)
    
    self._ballUniqueId = p.loadURDF("sphere2.urdf",[20,20,1])
    p.setGravity(0,0,-10)
    self._racecar = racecar.Racecar(urdfRootPath=self._urdfRoot, timeStep=self._timeStep)
    self._envStepCounter = 0
    for i in range(100):
      p.stepSimulation()
    self._observation = self._racecar.getObservation()
    return self._observation

  def __del__(self):
    p.disconnect()

  def _seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]

  def _step(self, action):
    if (self._render):
      basePos,orn = p.getBasePositionAndOrientation(self._racecar.racecarUniqueId)
      p.resetDebugVisualizerCamera(1, 30, -40, basePos)

    if len(action) != self._racecar.getActionDimension():
      raise ValueError("We expect {} continuous action not {}.".format(self._racecar.getActionDimension(), len(action)))

    for i in range(len(action)):
      if not -1.01 <= action[i] <= 1.01:
        raise ValueError("{}th action should be between -1 and 1 not {}.".format(i, action[i]))

    self._racecar.applyAction(action)
    for i in range(self._actionRepeat):
      p.stepSimulation()
      if self._render:
        time.sleep(self._timeStep)
      self._observation = self._racecar.getObservation()
      if self._termination():
        break
      self._envStepCounter += 1
    reward = self._reward()
    done = self._termination()
    return np.array(self._observation), reward, done, {}

  def _render(self, mode='human', close=False):
      return

  def _termination(self):
    return False
    
  def _reward(self):
    closestPoints = p.getClosestPoints(self._racecar.racecarUniqueId,self._ballUniqueId,10000) 
    numPt = len(closestPoints)
    reward=-1000
    print(numPt)
    if (numPt>0):
      print("reward:")
      reward = closestPoints[0][8]
      print(reward)
    return reward
