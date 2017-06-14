import pybullet as p
import numpy as np
import copy
import math

class Kuka:

  def __init__(self, urdfRootPath='', timeStep=0.01):
    self.urdfRootPath = urdfRootPath
    self.timeStep = timeStep
    self.reset()
    self.maxForce = 100

  def reset(self):
    
    objects = p.loadSDF("kuka_iiwa/kuka_with_gripper.sdf")
    self.kukaUid = objects[0]
    p.resetBasePositionAndOrientation(self.kukaUid,[-0.100000,0.000000,0.070000],[0.000000,0.000000,0.000000,1.000000])
    self.jointPositions=[ -0.196884, 0.857586, -0.023543, -1.664977, 0.030403, 0.624786, -0.232294, 0.000000, -0.296450, 0.000000, 0.100002, 0.284399, 0.000000, -0.099999 ]
    for jointIndex in range (p.getNumJoints(self.kukaUid)):
      p.resetJointState(self.kukaUid,jointIndex,self.jointPositions[jointIndex])
    
    self.trayUid = p.loadURDF("tray/tray.urdf", 0.640000,0.075000,-0.190000,0.000000,0.000000,1.000000,0.000000)
    self.blockUid =p.loadURDF("block.urdf", 0.604746,0.080626,-0.180069,0.000050,-0.000859,-0.824149,0.566372)

    self.motorNames = []
    self.motorIndices = []
    numJoints = p.getNumJoints(self.kukaUid)
    for i in range (numJoints):
      jointInfo = p.getJointInfo(self.kukaUid,i)
      qIndex = jointInfo[3]
      if qIndex > -1:
        print("motorname")
        print(jointInfo[1])
        self.motorNames.append(str(jointInfo[1]))
        self.motorIndices.append(i)

  def getActionDimension(self):
    return len(self.motorIndices)

  def getObservationDimension(self):
    return len(self.getObservation())

  def getObservation(self):
    observation = []
    pos,orn=p.getBasePositionAndOrientation(self.blockUid)
    
    observation.extend(list(pos))
    observation.extend(list(orn))
    
    return observation

  def applyAction(self, motorCommands):
    
    for action in range (len(motorCommands)):
      motor = self.motorIndices[action]
      p.setJointMotorControl2(self.kukaUid,motor,p.POSITION_CONTROL,targetPosition=motorCommands[action],force=self.maxForce)
    