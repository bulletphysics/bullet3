import pybullet as p

class Minitaur:
  def __init__(self):
    self.reset()

  def reset(self):
    self.quadruped = p.loadURDF("quadruped/quadruped.urdf",0,0,.3)
    self.kp = 1
    self.kd = 0.1
    self.maxForce = 100
    nJoints = p.getNumJoints(self.quadruped)
    self.jointNameToId = {}
    for i in range(nJoints):
      jointInfo = p.getJointInfo(self.quadruped, i)
      self.jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
    self.resetPose()
    for i in range(100):
      p.stepSimulation()

  def disableAllMotors(self):
    nJoints = p.getNumJoints(self.quadruped)
    for i in range(nJoints):
      p.setJointMotorControl2(bodyIndex=self.quadruped, jointIndex=i, controlMode=p.VELOCITY_CONTROL, force=0)

  def setMotorAngleByName(self, motorName, desiredAngle):
    p.setJointMotorControl2(bodyIndex=self.quadruped, jointIndex=self.jointNameToId[motorName], controlMode=p.POSITION_CONTROL, targetPosition=desiredAngle, positionGain=self.kp, velocityGain=self.kd, force=self.maxForce)

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
    self.setMotorAngleByName('motor_front_rightR_joint', motorCommands[0])
    self.setMotorAngleByName('motor_front_rightL_joint', motorCommands[1])
    self.setMotorAngleByName('motor_front_leftR_joint', motorCommands[2])
    self.setMotorAngleByName('motor_front_leftL_joint', motorCommands[3])
    self.setMotorAngleByName('motor_back_rightR_joint', motorCommands[4])
    self.setMotorAngleByName('motor_back_rightL_joint', motorCommands[5])
    self.setMotorAngleByName('motor_back_leftR_joint', motorCommands[6])
    self.setMotorAngleByName('motor_back_leftL_joint', motorCommands[7])
