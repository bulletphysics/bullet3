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
    self.motorIdList.append(self.jointNameToId['motor_front_leftL_joint'])
    self.motorIdList.append(self.jointNameToId['motor_front_leftR_joint'])
    self.motorIdList.append(self.jointNameToId['motor_back_leftL_joint'])
    self.motorIdList.append(self.jointNameToId['motor_back_leftR_joint'])
    self.motorIdList.append(self.jointNameToId['motor_front_rightL_joint'])
    self.motorIdList.append(self.jointNameToId['motor_front_rightR_joint'])
    self.motorIdList.append(self.jointNameToId['motor_back_rightL_joint'])
    self.motorIdList.append(self.jointNameToId['motor_back_rightR_joint'])


  def reset(self):
    self.quadruped = p.loadURDF("%s/quadruped/minitaur.urdf" % self.urdfRootPath,0,0,.2)
    self.kp = 1
    self.kd = 0.1
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
