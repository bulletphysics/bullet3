import pybullet as p
import numpy as np
import copy
import math

class Kuka:

  def __init__(self, urdfRootPath='', timeStep=0.01):
    self.urdfRootPath = urdfRootPath
    self.timeStep = timeStep
    self.reset()
    self.maxForce = 90
    self.fingerAForce = 12
    self.fingerBForce = 10
    self.fingerTipForce = 3
    
    self.useInverseKinematics = 1
    self.useSimulation = 1
    self.useNullSpace = 1
    self.useOrientation = 1
    self.kukaEndEffectorIndex = 6
    #lower limits for null space
    self.ll=[-.967,-2 ,-2.96,0.19,-2.96,-2.09,-3.05]
    #upper limits for null space
    self.ul=[.967,2 ,2.96,2.29,2.96,2.09,3.05]
    #joint ranges for null space
    self.jr=[5.8,4,5.8,4,5.8,4,6]
    #restposes for null space
    self.rp=[0,0,0,0.5*math.pi,0,-math.pi*0.5*0.66,0]
    #joint damping coefficents
    self.jd=[0.1,0.1,0.1,0.1,0.1,0.1,0.1]
    
  def reset(self):
    
    objects = p.loadSDF("kuka_iiwa/kuka_with_gripper2.sdf")
    self.kukaUid = objects[0]
    for i in range (p.getNumJoints(self.kukaUid)):
    	print(p.getJointInfo(self.kukaUid,i))
    p.resetBasePositionAndOrientation(self.kukaUid,[-0.100000,0.000000,0.070000],[0.000000,0.000000,0.000000,1.000000])
    self.jointPositions=[ -0.196884, 0.857586, -0.023543, -1.664977, 0.030403, 0.624786, -0.232294, 0.000000, -0.296450, 0.000000, 0.100002, 0.284399, 0.000000, -0.099999 ]
    self.numJoints = p.getNumJoints(self.kukaUid)
    for jointIndex in range (self.numJoints):
      p.resetJointState(self.kukaUid,jointIndex,self.jointPositions[jointIndex])
    
    self.trayUid = p.loadURDF("tray/tray.urdf", 0.640000,0.075000,-0.190000,0.000000,0.000000,1.000000,0.000000)
    self.blockUid =p.loadURDF("block.urdf", 0.604746,0.080626,-0.180069,0.000050,-0.000859,-0.824149,0.566372)
    p.loadURDF("table/table.urdf", 0.5000000,0.00000,-.820000,0.000000,0.000000,0.0,1.0)
    
    self.motorNames = []
    self.motorIndices = []
    
    for i in range (self.numJoints):
      jointInfo = p.getJointInfo(self.kukaUid,i)
      qIndex = jointInfo[3]
      if qIndex > -1:
        #print("motorname")
        #print(jointInfo[1])
        self.motorNames.append(str(jointInfo[1]))
        self.motorIndices.append(i)

  def getActionDimension(self):
    if (self.useInverseKinematics):
      return len(self.motorIndices)
    return 6 #position x,y,z and roll/pitch/yaw euler angles of end effector

  def getObservationDimension(self):
    return len(self.getObservation())

  def getObservation(self):
    observation = []
    pos,orn=p.getBasePositionAndOrientation(self.blockUid)
    
    observation.extend(list(pos))
    observation.extend(list(orn))
    
    return observation

  def applyAction(self, motorCommands):
    #print ("self.numJoints")
    #print (self.numJoints)
    if (self.useInverseKinematics):
      pos = [motorCommands[0],motorCommands[1],motorCommands[2]]
      yaw = motorCommands[3]
      fingerAngle = motorCommands[4]
      #roll = motorCommands[5]
      orn = p.getQuaternionFromEuler([0,-math.pi,0]) # -math.pi,yaw])
      if (self.useNullSpace==1):
        if (self.useOrientation==1):
          jointPoses = p.calculateInverseKinematics(self.kukaUid,self.kukaEndEffectorIndex,pos,orn,self.ll,self.ul,self.jr,self.rp)
        else:
          jointPoses = p.calculateInverseKinematics(self.kukaUid,self.kukaEndEffectorIndex,pos,lowerLimits=self.ll, upperLimits=self.ul, jointRanges=self.jr, restPoses=self.rp)
      else:
        if (self.useOrientation==1):
          jointPoses = p.calculateInverseKinematics(self.kukaUid,self.kukaEndEffectorIndex,pos,orn,jointDamping=self.jd)
        else:
          jointPoses = p.calculateInverseKinematics(self.kukaUid,self.kukaEndEffectorIndex,pos)
    
      #print("jointPoses")
      #print(jointPoses)
      #print("self.kukaEndEffectorIndex")
      #print(self.kukaEndEffectorIndex)
      if (self.useSimulation):
        for i in range (self.kukaEndEffectorIndex+1):
          #print(i)
          p.setJointMotorControl2(bodyIndex=self.kukaUid,jointIndex=i,controlMode=p.POSITION_CONTROL,targetPosition=jointPoses[i],targetVelocity=0,force=self.maxForce,positionGain=0.03,velocityGain=1)
      else:
        #reset the joint state (ignoring all dynamics, not recommended to use during simulation)
        for i in range (self.numJoints):
          p.resetJointState(self.kukaUid,i,jointPoses[i])
      #fingers
      p.setJointMotorControl2(self.kukaUid,7,p.POSITION_CONTROL,targetPosition=yaw,force=self.maxForce)
      p.setJointMotorControl2(self.kukaUid,8,p.POSITION_CONTROL,targetPosition=-fingerAngle,force=self.fingerAForce)
      p.setJointMotorControl2(self.kukaUid,11,p.POSITION_CONTROL,targetPosition=fingerAngle,force=self.fingerBForce)
      
      p.setJointMotorControl2(self.kukaUid,10,p.POSITION_CONTROL,targetPosition=0,force=self.fingerTipForce)
      p.setJointMotorControl2(self.kukaUid,13,p.POSITION_CONTROL,targetPosition=0,force=self.fingerTipForce)
      
      
    else:
      for action in range (len(motorCommands)):
        motor = self.motorIndices[action]
        p.setJointMotorControl2(self.kukaUid,motor,p.POSITION_CONTROL,targetPosition=motorCommands[action],force=self.maxForce)
      
