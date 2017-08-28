import os,  inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)

import math
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import time
import pybullet
from . import racecar
import random
from . import bullet_client
import pybullet_data

class RacecarGymEnv(gym.Env):
  metadata = {
      'render.modes': ['human', 'rgb_array'],
      'video.frames_per_second' : 50
  }

  def __init__(self,
               urdfRoot=pybullet_data.getDataPath(),
               actionRepeat=50,
               isEnableSelfCollision=True,
               isDiscrete=False,
               renders=False):
    print("init")
    self._timeStep = 0.01
    self._urdfRoot = urdfRoot
    self._actionRepeat = actionRepeat
    self._isEnableSelfCollision = isEnableSelfCollision
    self._observation = []
    self._ballUniqueId = -1
    self._envStepCounter = 0
    self._renders = renders
    self._isDiscrete = isDiscrete
    if self._renders:
      self._p = bullet_client.BulletClient(
          connection_mode=pybullet.GUI)
    else:
      self._p = bullet_client.BulletClient()

    self._seed()
    #self.reset()
    observationDim = 2 #len(self.getExtendedObservation())
    #print("observationDim")
    #print(observationDim) 
    # observation_high = np.array([np.finfo(np.float32).max] * observationDim)    
    observation_high = np.ones(observationDim) * 1000 #np.inf
    if (isDiscrete):
      self.action_space = spaces.Discrete(9)
    else:
       action_dim = 2
       self._action_bound = 1
       action_high = np.array([self._action_bound] * action_dim)
       self.action_space = spaces.Box(-action_high, action_high) 
    self.observation_space = spaces.Box(-observation_high, observation_high)
    self.viewer = None

  def _reset(self):
    self._p.resetSimulation()
    #p.setPhysicsEngineParameter(numSolverIterations=300)
    self._p.setTimeStep(self._timeStep)
    #self._p.loadURDF(os.path.join(self._urdfRoot,"plane.urdf"))
    stadiumobjects = self._p.loadSDF(os.path.join(self._urdfRoot,"stadium.sdf"))
    #move the stadium objects slightly above 0
    #for i in stadiumobjects:
    #	pos,orn = self._p.getBasePositionAndOrientation(i)
    #	newpos = [pos[0],pos[1],pos[2]-0.1]
    #	self._p.resetBasePositionAndOrientation(i,newpos,orn)
    
    dist = 5 +2.*random.random()
    ang = 2.*3.1415925438*random.random()
    
    ballx = dist * math.sin(ang)
    bally = dist * math.cos(ang)
    ballz = 1
    
    self._ballUniqueId = self._p.loadURDF(os.path.join(self._urdfRoot,"sphere2.urdf"),[ballx,bally,ballz])
    self._p.setGravity(0,0,-10)
    self._racecar = racecar.Racecar(self._p,urdfRootPath=self._urdfRoot, timeStep=self._timeStep)
    self._envStepCounter = 0
    for i in range(100):
      self._p.stepSimulation()
    self._observation = self.getExtendedObservation()
    return np.array(self._observation)

  def __del__(self):
    self._p = 0

  def _seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]

  def getExtendedObservation(self):
     self._observation = [] #self._racecar.getObservation()
     carpos,carorn = self._p.getBasePositionAndOrientation(self._racecar.racecarUniqueId)
     ballpos,ballorn = self._p.getBasePositionAndOrientation(self._ballUniqueId)      
     invCarPos,invCarOrn = self._p.invertTransform(carpos,carorn)
     ballPosInCar,ballOrnInCar = self._p.multiplyTransforms(invCarPos,invCarOrn,ballpos,ballorn)
    
     self._observation.extend([ballPosInCar[0],ballPosInCar[1]])
     return self._observation
     
  def _step(self, action):
    if (self._renders):
      basePos,orn = self._p.getBasePositionAndOrientation(self._racecar.racecarUniqueId)
      #self._p.resetDebugVisualizerCamera(1, 30, -40, basePos)
    
    if (self._isDiscrete):
	    fwd = [-1,-1,-1,0,0,0,1,1,1]
	    steerings = [-0.6,0,0.6,-0.6,0,0.6,-0.6,0,0.6]
	    forward = fwd[action]
	    steer = steerings[action]
	    realaction = [forward,steer]
    else:
      realaction = action

    self._racecar.applyAction(realaction)
    for i in range(self._actionRepeat):
      self._p.stepSimulation()
      if self._renders:
        time.sleep(self._timeStep)
      self._observation = self.getExtendedObservation()
      
      if self._termination():
        break
      self._envStepCounter += 1
    reward = self._reward()
    done = self._termination()
    #print("len=%r" % len(self._observation))
    
    return np.array(self._observation), reward, done, {}

  def _render(self, mode='human', close=False):
      width=320
      height=200
      img_arr = self._p.getCameraImage(width,height)
      w=img_arr[0]
      h=img_arr[1]
      rgb=img_arr[2]
      dep=img_arr[3]
      #print 'width = %d height = %d' % (w,h)
      # reshape creates np array
      np_img_arr = np.reshape(rgb, (h, w, 4))
      # remove alpha channel
      np_img_arr = np_img_arr[:, :, :3]
      return np_img_arr

  def _termination(self):
    return self._envStepCounter>1000
    
  def _reward(self):
    closestPoints = self._p.getClosestPoints(self._racecar.racecarUniqueId,self._ballUniqueId,10000) 
    
    numPt = len(closestPoints)
    reward=-1000
    #print(numPt)
    if (numPt>0):
      #print("reward:")
      reward = -closestPoints[0][8]
      #print(reward)
    return reward
