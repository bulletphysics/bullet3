import pybullet as p
import numpy as np

class Minitaur:
  def __init__(self, urdfRootPath=''):
    self.urdfRootPath = urdfRootPath
    self.reset()

  def buildJointNameToIdDict(self):
    nJoints = p.getNumJoints(self.quadruped)
    self.jointNameToId = {}
    for i in range(nJoints):
      jointInfo = p.getJointInfo(self.quadruped, i)
      self.jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
    self.resetPose()
    for i in range(100):
      p.stepSimulation()

  def buildMotorIdList(self):
    self.motorIdList.append(self.jointNameToId['motor_front_leftR_joint'])
    self.motorIdList.append(self.jointNameToId['motor_front_leftL_joint'])
    self.motorIdList.append(self.jointNameToId['motor_back_leftR_joint'])
    self.motorIdList.append(self.jointNameToId['motor_back_leftL_joint'])
    self.motorIdList.append(self.jointNameToId['motor_front_rightL_joint'])
    self.motorIdList.append(self.jointNameToId['motor_front_rightR_joint'])
    self.motorIdList.append(self.jointNameToId['motor_back_rightL_joint'])
    self.motorIdList.append(self.jointNameToId['motor_back_rightR_joint'])


  def reset(self):
    self.quadruped = p.loadURDF("%s/quadruped/quadruped.urdf" % self.urdfRootPath,0,0,.3)
    self.kp = 1
    self.kd = 0.1
    self.maxForce = 3.5
    self.nMotors = 8
    self.motorIdList = []
    self.motorDir = [1, -1, 1, -1, -1, 1, -1, 1]
    self.buildJointNameToIdDict()
    self.buildMotorIdList()


  def disableAllMotors(self):
    nJoints = p.getNumJoints(self.quadruped)
    for i in range(nJoints):
      p.setJointMotorControl2(bodyIndex=self.quadruped, jointIndex=i, controlMode=p.VELOCITY_CONTROL, force=0)

  def setMotorAngleById(self, motorId, desiredAngle):
    p.setJointMotorControl2(bodyIndex=self.quadruped, jointIndex=motorId, controlMode=p.POSITION_CONTROL, targetPosition=desiredAngle, positionGain=self.kp, velocityGain=self.kd, force=self.maxForce)

  def setMotorAngleByName(self, motorName, desiredAngle):
    self.setMotorAngleById(self.jointNameToId[motorName], desiredAngle)

  def resetPose(self):
    #right front leg
    self.disableAllMotors()
    p.resetJointState(self.quadruped,self.jointNameToId['motor_front_rightR_joint'],1.57)
    p.resetJointState(self.quadruped,self.jointNameToId['knee_front_rightR_link'],-2.2)
    p.resetJointState(self.quadruped,self.jointNameToId['motor_front_rightL_joint'],-1.57)
    p.resetJointState(self.quadruped,self.jointNameToId['knee_front_rightL_link'],2.2)
    p.createConstraint(self.quadruped,self.jointNameToId['knee_front_rightR_link'],self.quadruped,self.jointNameToId['knee_front_rightL_link'],p.JOINT_POINT2POINT,[0,0,0],[0,0.01,0.2],[0,-0.015,0.2])
    self.setMotorAngleByName('motor_front_rightR_joint', 1.57)
    self.setMotorAngleByName('motor_front_rightL_joint',-1.57)

    #left front leg
    p.resetJointState(self.quadruped,self.jointNameToId['motor_front_leftR_joint'],1.57)
    p.resetJointState(self.quadruped,self.jointNameToId['knee_front_leftR_link'],-2.2)
    p.resetJointState(self.quadruped,self.jointNameToId['motor_front_leftL_joint'],-1.57)
    p.resetJointState(self.quadruped,self.jointNameToId['knee_front_leftL_link'],2.2)
    p.createConstraint(self.quadruped,self.jointNameToId['knee_front_leftR_link'],self.quadruped,self.jointNameToId['knee_front_leftL_link'],p.JOINT_POINT2POINT,[0,0,0],[0,-0.01,0.2],[0,0.015,0.2])
    self.setMotorAngleByName('motor_front_leftR_joint', 1.57)
    self.setMotorAngleByName('motor_front_leftL_joint',-1.57)

    #right back leg
    p.resetJointState(self.quadruped,self.jointNameToId['motor_back_rightR_joint'],1.57)
    p.resetJointState(self.quadruped,self.jointNameToId['knee_back_rightR_link'],-2.2)
    p.resetJointState(self.quadruped,self.jointNameToId['motor_back_rightL_joint'],-1.57)
    p.resetJointState(self.quadruped,self.jointNameToId['knee_back_rightL_link'],2.2)
    p.createConstraint(self.quadruped,self.jointNameToId['knee_back_rightR_link'],self.quadruped,self.jointNameToId['knee_back_rightL_link'],p.JOINT_POINT2POINT,[0,0,0],[0,0.01,0.2],[0,-0.015,0.2])
    self.setMotorAngleByName('motor_back_rightR_joint', 1.57)
    self.setMotorAngleByName('motor_back_rightL_joint',-1.57)

    #left back leg
    p.resetJointState(self.quadruped,self.jointNameToId['motor_back_leftR_joint'],1.57)
    p.resetJointState(self.quadruped,self.jointNameToId['knee_back_leftR_link'],-2.2)
    p.resetJointState(self.quadruped,self.jointNameToId['motor_back_leftL_joint'],-1.57)
    p.resetJointState(self.quadruped,self.jointNameToId['knee_back_leftL_link'],2.2)
    p.createConstraint(self.quadruped,self.jointNameToId['knee_back_leftR_link'],self.quadruped,self.jointNameToId['knee_back_leftL_link'],p.JOINT_POINT2POINT,[0,0,0],[0,-0.01,0.2],[0,0.015,0.2])
    self.setMotorAngleByName('motor_back_leftR_joint', 1.57)
    self.setMotorAngleByName('motor_back_leftL_joint',-1.57)

  def getBasePosition(self):
    position, orientation = p.getBasePositionAndOrientation(self.quadruped)
    return position

  def getBaseOrientation(self):
    position, orientation = p.getBasePositionAndOrientation(self.quadruped)
    return orientation

  def applyAction(self, motorCommands):
    motorCommandsWithDir = np.multiply(motorCommands, self.motorDir)
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
