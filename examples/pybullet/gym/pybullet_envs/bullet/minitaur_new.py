import pybullet as p
import numpy as np
import copy
import math

class Minitaur:

  def __init__(self, urdfRootPath='', timeStep=0.01, isEnableSelfCollision=True, motorVelocityLimit=10.0):
    self.urdfRootPath = urdfRootPath
    self.isEnableSelfCollision = isEnableSelfCollision
    self.motorVelocityLimit = motorVelocityLimit
    self.timeStep = timeStep
    self.reset()

  def buildJointNameToIdDict(self):
    nJoints = p.getNumJoints(self.quadruped)
    self.jointNameToId = {}
    for i in range(nJoints):
      jointInfo = p.getJointInfo(self.quadruped, i)
      self.jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
    self.resetPose()

  def buildMotorIdList(self):
    self.motorIdList.append(self.jointNameToId['motor_front_leftL_joint'])
    self.motorIdList.append(self.jointNameToId['motor_front_leftR_joint'])
    self.motorIdList.append(self.jointNameToId['motor_back_leftL_joint'])
    self.motorIdList.append(self.jointNameToId['motor_back_leftR_joint'])
    self.motorIdList.append(self.jointNameToId['motor_front_rightL_joint'])
    self.motorIdList.append(self.jointNameToId['motor_front_rightR_joint'])
    self.motorIdList.append(self.jointNameToId['motor_back_rightL_joint'])
    self.motorIdList.append(self.jointNameToId['motor_back_rightR_joint'])

  def reset(self):
    if self.isEnableSelfCollision:
      self.quadruped = p.loadURDF("%s/quadruped/minitaur.urdf" % self.urdfRootPath, [0,0,.2], flags=p.URDF_USE_SELF_COLLISION)
    else:
      self.quadruped = p.loadURDF("%s/quadruped/minitaur.urdf" % self.urdfRootPath, [0,0,.2])
    self.kp = 1
    self.kd = 1
    self.maxForce = 3.5
    self.nMotors = 8
    self.motorIdList = []
    self.motorDir = [-1, -1, -1, -1, 1, 1, 1, 1]
    self.buildJointNameToIdDict()
    self.buildMotorIdList()


  def setMotorAngleById(self, motorId, desiredAngle):
    p.setJointMotorControl2(bodyIndex=self.quadruped, jointIndex=motorId, controlMode=p.POSITION_CONTROL, targetPosition=desiredAngle, positionGain=self.kp, velocityGain=self.kd, force=self.maxForce)

  def setMotorAngleByName(self, motorName, desiredAngle):
    self.setMotorAngleById(self.jointNameToId[motorName], desiredAngle)

  def resetPose(self):
    kneeFrictionForce = 0
    halfpi = 1.57079632679
    kneeangle = -2.1834 #halfpi - acos(upper_leg_length / lower_leg_length)

    #left front leg
    p.resetJointState(self.quadruped,self.jointNameToId['motor_front_leftL_joint'],self.motorDir[0]*halfpi)
    p.resetJointState(self.quadruped,self.jointNameToId['knee_front_leftL_link'],self.motorDir[0]*kneeangle)
    p.resetJointState(self.quadruped,self.jointNameToId['motor_front_leftR_joint'],self.motorDir[1]*halfpi)
    p.resetJointState(self.quadruped,self.jointNameToId['knee_front_leftR_link'],self.motorDir[1]*kneeangle)
    p.createConstraint(self.quadruped,self.jointNameToId['knee_front_leftR_link'],self.quadruped,self.jointNameToId['knee_front_leftL_link'],p.JOINT_POINT2POINT,[0,0,0],[0,0.005,0.2],[0,0.01,0.2])
    self.setMotorAngleByName('motor_front_leftL_joint', self.motorDir[0]*halfpi)
    self.setMotorAngleByName('motor_front_leftR_joint', self.motorDir[1]*halfpi)
    p.setJointMotorControl2(bodyIndex=self.quadruped,jointIndex=self.jointNameToId['knee_front_leftL_link'],controlMode=p.VELOCITY_CONTROL,targetVelocity=0,force=kneeFrictionForce)
    p.setJointMotorControl2(bodyIndex=self.quadruped,jointIndex=self.jointNameToId['knee_front_leftR_link'],controlMode=p.VELOCITY_CONTROL,targetVelocity=0,force=kneeFrictionForce)

    #left back leg
    p.resetJointState(self.quadruped,self.jointNameToId['motor_back_leftL_joint'],self.motorDir[2]*halfpi)
    p.resetJointState(self.quadruped,self.jointNameToId['knee_back_leftL_link'],self.motorDir[2]*kneeangle)
    p.resetJointState(self.quadruped,self.jointNameToId['motor_back_leftR_joint'],self.motorDir[3]*halfpi)
    p.resetJointState(self.quadruped,self.jointNameToId['knee_back_leftR_link'],self.motorDir[3]*kneeangle)
    p.createConstraint(self.quadruped,self.jointNameToId['knee_back_leftR_link'],self.quadruped,self.jointNameToId['knee_back_leftL_link'],p.JOINT_POINT2POINT,[0,0,0],[0,0.005,0.2],[0,0.01,0.2])
    self.setMotorAngleByName('motor_back_leftL_joint',self.motorDir[2]*halfpi)
    self.setMotorAngleByName('motor_back_leftR_joint',self.motorDir[3]*halfpi)
    p.setJointMotorControl2(bodyIndex=self.quadruped,jointIndex=self.jointNameToId['knee_back_leftL_link'],controlMode=p.VELOCITY_CONTROL,targetVelocity=0,force=kneeFrictionForce)
    p.setJointMotorControl2(bodyIndex=self.quadruped,jointIndex=self.jointNameToId['knee_back_leftR_link'],controlMode=p.VELOCITY_CONTROL,targetVelocity=0,force=kneeFrictionForce)
 
    #right front leg
    p.resetJointState(self.quadruped,self.jointNameToId['motor_front_rightL_joint'],self.motorDir[4]*halfpi)
    p.resetJointState(self.quadruped,self.jointNameToId['knee_front_rightL_link'],self.motorDir[4]*kneeangle)
    p.resetJointState(self.quadruped,self.jointNameToId['motor_front_rightR_joint'],self.motorDir[5]*halfpi)
    p.resetJointState(self.quadruped,self.jointNameToId['knee_front_rightR_link'],self.motorDir[5]*kneeangle)
    p.createConstraint(self.quadruped,self.jointNameToId['knee_front_rightR_link'],self.quadruped,self.jointNameToId['knee_front_rightL_link'],p.JOINT_POINT2POINT,[0,0,0],[0,0.005,0.2],[0,0.01,0.2])
    self.setMotorAngleByName('motor_front_rightL_joint',self.motorDir[4]*halfpi)
    self.setMotorAngleByName('motor_front_rightR_joint',self.motorDir[5]*halfpi)
    p.setJointMotorControl2(bodyIndex=self.quadruped,jointIndex=self.jointNameToId['knee_front_rightL_link'],controlMode=p.VELOCITY_CONTROL,targetVelocity=0,force=kneeFrictionForce)
    p.setJointMotorControl2(bodyIndex=self.quadruped,jointIndex=self.jointNameToId['knee_front_rightR_link'],controlMode=p.VELOCITY_CONTROL,targetVelocity=0,force=kneeFrictionForce)
 

    #right back leg
    p.resetJointState(self.quadruped,self.jointNameToId['motor_back_rightL_joint'],self.motorDir[6]*halfpi)
    p.resetJointState(self.quadruped,self.jointNameToId['knee_back_rightL_link'],self.motorDir[6]*kneeangle)
    p.resetJointState(self.quadruped,self.jointNameToId['motor_back_rightR_joint'],self.motorDir[7]*halfpi)
    p.resetJointState(self.quadruped,self.jointNameToId['knee_back_rightR_link'],self.motorDir[7]*kneeangle)
    p.createConstraint(self.quadruped,self.jointNameToId['knee_back_rightR_link'],self.quadruped,self.jointNameToId['knee_back_rightL_link'],p.JOINT_POINT2POINT,[0,0,0],[0,0.005,0.2],[0,0.01,0.2])
    self.setMotorAngleByName('motor_back_rightL_joint',self.motorDir[6]*halfpi)
    self.setMotorAngleByName('motor_back_rightR_joint',self.motorDir[7]*halfpi)
    p.setJointMotorControl2(bodyIndex=self.quadruped,jointIndex=self.jointNameToId['knee_back_rightL_link'],controlMode=p.VELOCITY_CONTROL,targetVelocity=0,force=kneeFrictionForce)
    p.setJointMotorControl2(bodyIndex=self.quadruped,jointIndex=self.jointNameToId['knee_back_rightR_link'],controlMode=p.VELOCITY_CONTROL,targetVelocity=0,force=kneeFrictionForce)


  def getBasePosition(self):
    position, orientation = p.getBasePositionAndOrientation(self.quadruped)
    return position

  def getBaseOrientation(self):
    position, orientation = p.getBasePositionAndOrientation(self.quadruped)
    return orientation

  def getActionDimension(self):
    return self.nMotors

  def getObservationDimension(self):
    return len(self.getObservation())

  def getObservation(self):
    observation = []
    observation.extend(self.getMotorAngles().tolist())
    observation.extend(self.getMotorVelocities().tolist())
    observation.extend(self.getMotorTorques().tolist())
    observation.extend(list(self.getBaseOrientation()))
    return observation


  def applyAction(self, motorCommands):
    if self.motorVelocityLimit < np.inf:
      currentMotorAngle = self.getMotorAngles()
      motorCommandsMax = currentMotorAngle + self.timeStep * self.motorVelocityLimit
      motorCommandsMin = currentMotorAngle - self.timeStep * self.motorVelocityLimit
      motorCommands = np.clip(motorCommands, motorCommandsMin, motorCommandsMax)
    motorCommandsWithDir = np.multiply(motorCommands, self.motorDir)
#    print('action: {}'.format(motorCommands))
#    print('motor: {}'.format(motorCommandsWithDir))
    for i in range(self.nMotors):
      self.setMotorAngleById(self.motorIdList[i], motorCommandsWithDir[i])

  def getMotorAngles(self):
    motorAngles = []
    for i in range(self.nMotors):
      jointState = p.getJointState(self.quadruped, self.motorIdList[i])
      motorAngles.append(jointState[0])
    motorAngles = np.multiply(motorAngles, self.motorDir)
    return motorAngles

  def getMotorVelocities(self):
    motorVelocities = []
    for i in range(self.nMotors):
      jointState = p.getJointState(self.quadruped, self.motorIdList[i])
      motorVelocities.append(jointState[1])
    motorVelocities = np.multiply(motorVelocities, self.motorDir)
    return motorVelocities

  def getMotorTorques(self):
    motorTorques = []
    for i in range(self.nMotors):
      jointState = p.getJointState(self.quadruped, self.motorIdList[i])
      motorTorques.append(jointState[3])
    motorTorques = np.multiply(motorTorques, self.motorDir)
    return motorTorques

  def convertFromLegModel(self, actions):
    motorAngle = copy.deepcopy(actions)
    scaleForSingularity = 1
    offsetForSingularity = 0.5
    motorAngle[0] = math.pi + math.pi / 4 * actions[0] - scaleForSingularity * math.pi / 4 * (actions[4] + 1 + offsetForSingularity)
    motorAngle[1] = math.pi - math.pi / 4 * actions[0] - scaleForSingularity * math.pi / 4 * (actions[4] + 1 + offsetForSingularity)
    motorAngle[2] = math.pi + math.pi / 4 * actions[1] - scaleForSingularity * math.pi / 4 * (actions[5] + 1 + offsetForSingularity)
    motorAngle[3] = math.pi - math.pi / 4 * actions[1] - scaleForSingularity * math.pi / 4 * (actions[5] + 1 + offsetForSingularity)
    motorAngle[4] = math.pi - math.pi / 4 * actions[2] - scaleForSingularity * math.pi / 4 * (actions[6] + 1 + offsetForSingularity)
    motorAngle[5] = math.pi + math.pi / 4 * actions[2] - scaleForSingularity * math.pi / 4 * (actions[6] + 1 + offsetForSingularity)
    motorAngle[6] = math.pi - math.pi / 4 * actions[3] - scaleForSingularity * math.pi / 4 * (actions[7] + 1 + offsetForSingularity)
    motorAngle[7] = math.pi + math.pi / 4 * actions[3] - scaleForSingularity * math.pi / 4 * (actions[7] + 1 + offsetForSingularity)
    return motorAngle
