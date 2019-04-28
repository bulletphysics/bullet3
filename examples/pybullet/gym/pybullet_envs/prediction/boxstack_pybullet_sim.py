"""This file implements the functionalities of an example simulated system using pybullet.

"""
import copy
import math
import numpy as np
import os
import pybullet_data
import random


class BoxStackPyBulletSim(object):
  """The ExamplePyBulletSim class that adds some objects to the scene, steps the sim and return a reward.

  """

  def __init__(self, pybullet_client, urdf_root=pybullet_data.getDataPath(), time_step=0.01):
    """Constructs an example simulation and reset it to the initial states.

    Args:
      pybullet_client: The instance of BulletClient to manage different
        simulations.
      urdf_root: The path to the urdf folder.
      time_step: The time step of the simulation.
    """
    self._pybullet_client = pybullet_client
    self._urdf_root = urdf_root
    self.m_actions_taken_since_reset = 0
    self.time_step = time_step
    self.stateId = -1
    self.Reset(reload_urdf=True)

  def Reset(self, reload_urdf=False):
    """Reset the minitaur to its initial states.

    Args:
      reload_urdf: Whether to reload the urdf file. If not, Reset() just place
        the minitaur back to its starting position.
    """
    self.m_actions_taken_since_reset = 0
    xPosRange = 0.025
    yPosRange = 0.025
    boxHalfExtents = 0.025

    if reload_urdf:
      camInfo = self._pybullet_client.getDebugVisualizerCamera()
      cameraDistance = camInfo[10]
      print("cameraYaw=", camInfo[8])
      print("cameraPitch=", camInfo[9])
      print("camtarget=", camInfo[11])
      print("projectionMatrix=", camInfo[3])
      self._pybullet_client.resetDebugVisualizerCamera(cameraDistance=0.3,
                                                       cameraYaw=camInfo[8],
                                                       cameraPitch=camInfo[9],
                                                       cameraTargetPosition=camInfo[11])

      plane = self._pybullet_client.loadURDF("plane.urdf")
      texUid = self._pybullet_client.loadTexture("checker_blue.png")
      self._pybullet_client.changeVisualShape(plane, -1, textureUniqueId=texUid)

      self._numObjects = 4  #random number?

      self._cubes = []

      red = [0.97, 0.25, 0.25, 1]
      green = [0.41, 0.68, 0.31, 1]
      yellow = [0.92, 0.73, 0, 1]
      blue = [0, 0.55, 0.81, 1]
      colors = [red, green, yellow, blue]

      for i in range(self._numObjects):
        pos = [0, 0, boxHalfExtents + i * 2 * boxHalfExtents]
        orn = self._pybullet_client.getQuaternionFromEuler([0, 0, 0])
        orn = [0, 0, 0, 1]
        cube = self._pybullet_client.loadURDF("cube_small.urdf", pos, orn)
        self._pybullet_client.changeVisualShape(cube, -1, rgbaColor=colors[i])
        self._cubes.append(cube)

      self._pybullet_client.setGravity(0, 0, -10)
      self.stateId = self._pybullet_client.saveState()
    else:
      if (self.stateId >= 0):
        self._pybullet_client.restoreState(self.stateId)
    index = 0
    for i in self._cubes:
      posX = random.uniform(-xPosRange, xPosRange)
      posY = random.uniform(-yPosRange, yPosRange)
      yaw = random.uniform(-math.pi, math.pi)
      pos = [posX, posY, boxHalfExtents + index * 2 * boxHalfExtents]
      index += 1
      orn = self._pybullet_client.getQuaternionFromEuler([0, 0, yaw])
      self._pybullet_client.resetBasePositionAndOrientation(i, pos, orn)

  def GetActionDimension(self):
    """Get the length of the action list.

    Returns:
      The length of the action list.
    """
    return 4  #self.num_motors

  def GetObservationUpperBound(self):
    """Get the upper bound of the observation.

    Returns:
      The upper bound of an observation. See GetObservation() for the details
        of each element of an observation.
    """
    upper_bound = np.array([0.0] * self.GetObservationDimension())
    upper_bound[0:] = 1.0  # Quaternion of base orientation.
    return upper_bound

  def GetObservationLowerBound(self):
    """Get the lower bound of the observation."""
    return -self.GetObservationUpperBound()

  def GetObservationDimension(self):
    """Get the length of the observation list.

    Returns:
      The length of the observation list.
    """
    sz = len(self.GetObservation())
    print("sz=", sz)
    return sz

  def GetObservation(self):
    """Get the observations of minitaur.

    Returns:
      The observation list. observation[0:4] is the base orientation in quaternion form.
    """
    observation = []
    for i in self._cubes:
      pos, orn = self._pybullet_client.getBasePositionAndOrientation(i)
      observation.extend(list(pos))
      observation.extend(list(orn))
    return observation

  def ApplyAction(self, action):
    """Set the desired action.
    """
    self.m_actions_taken_since_reset += 1

  def Termination(self):
    return self.m_actions_taken_since_reset >= 200


def CreateSim(pybullet_client, urdf_root, time_step):
  sim = BoxStackPyBulletSim(pybullet_client=pybullet_client,
                            urdf_root=urdf_root,
                            time_step=time_step)
  return sim
