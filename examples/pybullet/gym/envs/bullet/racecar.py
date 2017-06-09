import pybullet as p
import numpy as np
import copy
import math

class Racecar:

  def __init__(self, urdfRootPath='', timeStep=0.01):
    self.urdfRootPath = urdfRootPath
    self.timeStep = timeStep
    self.reset()

  def reset(self):
    self.racecarUniqueId = p.loadURDF("racecar/racecar.urdf", [0,0,.2])
    self.maxForce = 10
    self.nMotors = 2
    self.motorizedwheels=[2]
    self.inactiveWheels = [3,5,7]
    for wheel in self.inactiveWheels:
      p.setJointMotorControl2(self.racecarUniqueId,wheel,p.VELOCITY_CONTROL,targetVelocity=0,force=0)

    self.motorizedWheels = [2]
    self.steeringLinks=[4,6]
    self.speedMultiplier = 10.
		

  def getActionDimension(self):
    return self.nMotors

  def getObservationDimension(self):
    return len(self.getObservation())

  def getObservation(self):
    observation = []
    pos,orn=p.getBasePositionAndOrientation(self.racecarUniqueId)
    observation.extend(list(pos))
    return observation

  def applyAction(self, motorCommands):
    targetVelocity=motorCommands[0]*self.speedMultiplier
    print("targetVelocity")
    print(targetVelocity)
    steeringAngle = motorCommands[1]
    print("steeringAngle")
    print(steeringAngle)
    print("maxForce")
    print(self.maxForce)
    
    
    for motor in self.motorizedwheels:
	    p.setJointMotorControl2(self.racecarUniqueId,motor,p.VELOCITY_CONTROL,targetVelocity=targetVelocity,force=self.maxForce)		
    for steer in self.steeringLinks:
      p.setJointMotorControl2(self.racecarUniqueId,steer,p.POSITION_CONTROL,targetPosition=steeringAngle)
	