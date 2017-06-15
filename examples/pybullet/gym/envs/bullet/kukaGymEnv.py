import math
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import time
import pybullet as p
from . import kuka
import random

class KukaGymEnv(gym.Env):
  metadata = {
      'render.modes': ['human', 'rgb_array'],
      'video.frames_per_second' : 50
  }

  def __init__(self,
               urdfRoot="",
               actionRepeat=1,
               isEnableSelfCollision=True,
               renders=True):
    print("init")
    self._timeStep = 1./240.
    self._urdfRoot = urdfRoot
    self._actionRepeat = actionRepeat
    self._isEnableSelfCollision = isEnableSelfCollision
    self._observation = []
    self._envStepCounter = 0
    self._renders = renders
    self._p = p
    if self._renders:
      p.connect(p.GUI)
    else:
      p.connect(p.DIRECT)
    #timinglog = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "kukaTimings.json")
    self._seed()
    self.reset()
    observationDim = len(self.getExtendedObservation())
    #print("observationDim")
    #print(observationDim)
    
    observation_high = np.array([np.finfo(np.float32).max] * observationDim)    
    self.action_space = spaces.Discrete(7)
    self.observation_space = spaces.Box(-observation_high, observation_high)
    self.viewer = None

  def _reset(self):
    p.resetSimulation()
    p.setPhysicsEngineParameter(numSolverIterations=150)
    p.setTimeStep(self._timeStep)
    p.loadURDF("%splane.urdf" % self._urdfRoot,[0,0,-1])
    if self._renders:
      p.resetDebugVisualizerCamera(1.3,180,-41,[0.52,-0.2,-0.33])
    p.loadURDF("table/table.urdf", 0.5000000,0.00000,-.820000,0.000000,0.000000,0.0,1.0)
    
    xpos = 0.5 +0.25*random.random()
    ypos = 0 +0.22*random.random()
    ang = 3.1415925438*random.random()
    orn = p.getQuaternionFromEuler([0,0,ang])
    self.blockUid =p.loadURDF("block.urdf", xpos,ypos,0.02,orn[0],orn[1],orn[2],orn[3])
            
    p.setGravity(0,0,-10)
    self._kuka = kuka.Kuka(urdfRootPath=self._urdfRoot, timeStep=self._timeStep)
    self._envStepCounter = 0
    p.stepSimulation()
    self._observation = self.getExtendedObservation()
    return np.array(self._observation)

  def __del__(self):
    p.disconnect()

  def _seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]

  def getExtendedObservation(self):
     self._observation = self._kuka.getObservation()
     pos,orn = p.getBasePositionAndOrientation(self.blockUid)
     self._observation.extend(list(pos))
     self._observation.extend(list(orn))
      
     return self._observation
  
  def _step(self, action):
    dv = 0.005
    dx = [0,-dv,dv,0,0,0,0][action]
    dy = [0,0,0,-dv,dv,0,0][action]
    da = [0,0,0,0,0,-dv,dv][action]
    f = 0.3
    realAction = [dx,dy,-0.1,da,f]
    return self.step2( realAction)
     
  def step2(self, action):
    self._kuka.applyAction(action)
    for i in range(self._actionRepeat):
      p.stepSimulation()
      if self._renders:
        time.sleep(self._timeStep)
      self._observation = self.getExtendedObservation()
      if self._termination():
        break
      self._envStepCounter += 1
    #print("self._envStepCounter")
    #print(self._envStepCounter)
    
    done = self._termination()
    reward = self._reward()
    #print("len=%r" % len(self._observation))
    
    return np.array(self._observation), reward, done, {}

  def _render(self, mode='human', close=False):
      return

  def _termination(self):
    #print (self._kuka.endEffectorPos[2])
    if (self._envStepCounter>1000):
      for i in range (1000):
        p.stepSimulation()
      #start grasp and terminate
      fingerAngle = 0.3
      
      
      for i in range (5000):
        graspAction = [0,0,0,0,fingerAngle]
        self._kuka.applyAction(graspAction)
        p.stepSimulation()
        fingerAngle = fingerAngle-(0.3/5000.)
        
      for i in range (5000):
        graspAction = [0,0,0.0001,0,0]
        self._kuka.applyAction(graspAction)
        p.stepSimulation()
        fingerAngle = fingerAngle-(0.3/10000.)
        
      self._observation = self.getExtendedObservation()
      return True
    return False
    
  def _reward(self):
    
    #rewards is height of target object
    pos,orn=p.getBasePositionAndOrientation(self.blockUid)
    reward = pos[2]
    if (reward>0.2):
    	print("reward")
    	print(reward)
    return reward