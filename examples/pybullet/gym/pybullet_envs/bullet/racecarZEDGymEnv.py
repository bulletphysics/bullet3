import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import math
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import time
import pybullet
from pybullet_utils import bullet_client as bc
from . import racecar
import random
import pybullet_data
from pkg_resources import parse_version

RENDER_HEIGHT = 720
RENDER_WIDTH = 960


class RacecarZEDGymEnv(gym.Env):
  metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}

  def __init__(self,
               urdfRoot=pybullet_data.getDataPath(),
               actionRepeat=10,
               isEnableSelfCollision=True,
               isDiscrete=False,
               renders=True):
    print("init")
    self._timeStep = 0.01
    self._urdfRoot = urdfRoot
    self._actionRepeat = actionRepeat
    self._isEnableSelfCollision = isEnableSelfCollision
    self._ballUniqueId = -1
    self._envStepCounter = 0
    self._renders = renders
    self._width = 100
    self._height = 10

    self._isDiscrete = isDiscrete
    if self._renders:
      self._p = bc.BulletClient(connection_mode=pybullet.GUI)
    else:
      self._p = bc.BulletClient()

    self.seed()
    self.reset()
    observationDim = len(self.getExtendedObservation())
    #print("observationDim")
    #print(observationDim)

    observation_high = np.array([np.finfo(np.float32).max] * observationDim)
    if (isDiscrete):
      self.action_space = spaces.Discrete(9)
    else:
      action_dim = 2
      self._action_bound = 1
      action_high = np.array([self._action_bound] * action_dim)
      self.action_space = spaces.Box(-action_high, action_high, dtype=np.float32)
    self.observation_space = spaces.Box(low=0,
                                        high=255,
                                        shape=(self._height, self._width, 4),
                                        dtype=np.uint8)

    self.viewer = None

  def reset(self):
    self._p.resetSimulation()
    #p.setPhysicsEngineParameter(numSolverIterations=300)
    self._p.setTimeStep(self._timeStep)
    #self._p.loadURDF(os.path.join(os.path.dirname(__file__),"../data","plane.urdf"))
    stadiumobjects = self._p.loadSDF(os.path.join(self._urdfRoot, "stadium.sdf"))
    #move the stadium objects slightly above 0
    for i in stadiumobjects:
      pos, orn = self._p.getBasePositionAndOrientation(i)
      newpos = [pos[0], pos[1], pos[2] + 0.1]
      self._p.resetBasePositionAndOrientation(i, newpos, orn)

    dist = 5 + 2. * random.random()
    ang = 2. * 3.1415925438 * random.random()

    ballx = dist * math.sin(ang)
    bally = dist * math.cos(ang)
    ballz = 1

    self._ballUniqueId = self._p.loadURDF(os.path.join(self._urdfRoot, "sphere2red.urdf"),
                                          [ballx, bally, ballz])
    self._p.setGravity(0, 0, -10)
    self._racecar = racecar.Racecar(self._p, urdfRootPath=self._urdfRoot, timeStep=self._timeStep)
    self._envStepCounter = 0
    for i in range(100):
      self._p.stepSimulation()
    self._observation = self.getExtendedObservation()
    return np.array(self._observation)

  def __del__(self):
    self._p = 0

  def seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]

  def getExtendedObservation(self):
    carpos, carorn = self._p.getBasePositionAndOrientation(self._racecar.racecarUniqueId)
    carmat = self._p.getMatrixFromQuaternion(carorn)
    ballpos, ballorn = self._p.getBasePositionAndOrientation(self._ballUniqueId)
    invCarPos, invCarOrn = self._p.invertTransform(carpos, carorn)
    ballPosInCar, ballOrnInCar = self._p.multiplyTransforms(invCarPos, invCarOrn, ballpos, ballorn)
    dist0 = 0.3
    dist1 = 1.
    eyePos = [
        carpos[0] + dist0 * carmat[0], carpos[1] + dist0 * carmat[3],
        carpos[2] + dist0 * carmat[6] + 0.3
    ]
    targetPos = [
        carpos[0] + dist1 * carmat[0], carpos[1] + dist1 * carmat[3],
        carpos[2] + dist1 * carmat[6] + 0.3
    ]
    up = [carmat[2], carmat[5], carmat[8]]
    viewMat = self._p.computeViewMatrix(eyePos, targetPos, up)
    #viewMat = self._p.computeViewMatrixFromYawPitchRoll(carpos,1,0,0,0,2)
    #print("projectionMatrix:")
    #print(self._p.getDebugVisualizerCamera()[3])
    projMatrix = [
        0.7499999403953552, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0000200271606445, -1.0,
        0.0, 0.0, -0.02000020071864128, 0.0
    ]
    img_arr = self._p.getCameraImage(width=self._width,
                                     height=self._height,
                                     viewMatrix=viewMat,
                                     projectionMatrix=projMatrix)
    rgb = img_arr[2]
    np_img_arr = np.reshape(rgb, (self._height, self._width, 4))
    self._observation = np_img_arr
    return self._observation

  def step(self, action):
    if (self._renders):
      basePos, orn = self._p.getBasePositionAndOrientation(self._racecar.racecarUniqueId)
      #self._p.resetDebugVisualizerCamera(1, 30, -40, basePos)

    if (self._isDiscrete):
      fwd = [-1, -1, -1, 0, 0, 0, 1, 1, 1]
      steerings = [-0.6, 0, 0.6, -0.6, 0, 0.6, -0.6, 0, 0.6]
      forward = fwd[action]
      steer = steerings[action]
      realaction = [forward, steer]
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

  def render(self, mode='human', close=False):
    if mode != "rgb_array":
      return np.array([])
    base_pos, orn = self._p.getBasePositionAndOrientation(self._racecar.racecarUniqueId)
    view_matrix = self._p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=base_pos,
                                                            distance=self._cam_dist,
                                                            yaw=self._cam_yaw,
                                                            pitch=self._cam_pitch,
                                                            roll=0,
                                                            upAxisIndex=2)
    proj_matrix = self._p.computeProjectionMatrixFOV(fov=60,
                                                     aspect=float(RENDER_WIDTH) / RENDER_HEIGHT,
                                                     nearVal=0.1,
                                                     farVal=100.0)
    (_, _, px, _, _) = self._p.getCameraImage(width=RENDER_WIDTH,
                                              height=RENDER_HEIGHT,
                                              viewMatrix=view_matrix,
                                              projectionMatrix=proj_matrix,
                                              renderer=pybullet.ER_BULLET_HARDWARE_OPENGL)
    rgb_array = np.array(px)
    rgb_array = rgb_array[:, :, :3]
    return rgb_array

  def _termination(self):
    return self._envStepCounter > 1000

  def _reward(self):
    closestPoints = self._p.getClosestPoints(self._racecar.racecarUniqueId, self._ballUniqueId,
                                             10000)

    numPt = len(closestPoints)
    reward = -1000
    #print(numPt)
    if (numPt > 0):
      #print("reward:")
      reward = -closestPoints[0][8]
      #print(reward)
    return reward

  if parse_version(gym.__version__) < parse_version('0.9.6'):
    _render = render
    _reset = reset
    _seed = seed
    _step = step
