import os
import numpy as np
import copy
import math

class Racecar:

  def __init__(self, bullet_client, urdfRootPath='', timeStep=0.01):
    self.urdfRootPath = urdfRootPath
    self.timeStep = timeStep
    self._p = bullet_client
    self.reset()

  def reset(self):
    self.racecarUniqueId = self._p.loadURDF(os.path.join(os.path.dirname(__file__),"../data","racecar/racecar.urdf"), [0,0,.2])
    self.maxForce = 20
    self.nMotors = 2
    self.motorizedwheels=[2]
    self.inactiveWheels = [3,5,7]
    for wheel in self.inactiveWheels:
      self._p.setJointMotorControl2(self.racecarUniqueId,wheel,self._p.VELOCITY_CONTROL,targetVelocity=0,force=0)

    self.motorizedWheels = [2]
    self.steeringLinks=[4,6]
    self.speedMultiplier = 4.
    

  def getActionDimension(self):
    return self.nMotors

  def getObservationDimension(self):
    return len(self.getObservation())

  def getObservation(self):
    observation = []
    pos,orn=self._p.getBasePositionAndOrientation(self.racecarUniqueId)
    
    observation.extend(list(pos))
    observation.extend(list(orn))
    
    return observation

  def applyAction(self, motorCommands):
    targetVelocity=motorCommands[0]*self.speedMultiplier
    #print("targetVelocity")
    #print(targetVelocity)
    steeringAngle = motorCommands[1]
    #print("steeringAngle")
    #print(steeringAngle)
    #print("maxForce")
    #print(self.maxForce)
    
    
    for motor in self.motorizedwheels:
      self._p.setJointMotorControl2(self.racecarUniqueId,motor,self._p.VELOCITY_CONTROL,targetVelocity=targetVelocity,force=self.maxForce)    
    for steer in self.steeringLinks:
      self._p.setJointMotorControl2(self.racecarUniqueId,steer,self._p.POSITION_CONTROL,targetPosition=steeringAngle)
  
