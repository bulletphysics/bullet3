import os,  inspect
import math
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)

class HumanoidPose(object):
  def __init__(self):
    pass
  
  def Reset(self):
    
    self._basePos = [0,0,0]
    self._baseLinVel = [0,0,0]
    self._baseOrn = [0,0,0,1]
    self._baseAngVel = [0,0,0]
    
    self._chestRot = [0,0,0,1]
    self._chestVel = [0,0,0]
    self._neckRot = [0,0,0,1]
    self._neckVel = [0,0,0]
    
    self._rightHipRot = [0,0,0,1]
    self._rightHipVel = [0,0,0]
    self._rightKneeRot = [0]
    self._rightKneeVel = [0]
    self._rightAnkleRot = [0,0,0,1]
    self._rightAnkleVel = [0,0,0]
    
    self._rightShoulderRot = [0,0,0,1]
    self._rightShoulderVel = [0,0,0]
    self._rightElbowRot = [0]
    self._rightElbowVel = [0]

    self._leftHipRot = [0,0,0,1]
    self._leftHipVel = [0,0,0]
    self._leftKneeRot = [0]
    self._leftKneeVel = [0]
    self._leftAnkleRot = [0,0,0,1]
    self._leftAnkleVel = [0,0,0]
    
    self._leftShoulderRot = [0,0,0,1]
    self._leftShoulderVel = [0,0,0]
    self._leftElbowRot = [0]
    self._leftElbowVel = [0]

  def ComputeLinVel(self,posStart, posEnd, deltaTime):
    vel = [(posEnd[0]-posStart[0])/deltaTime,(posEnd[1]-posStart[1])/deltaTime,(posEnd[2]-posStart[2])/deltaTime]
    return vel
  
  def ComputeAngVel(self,ornStart, ornEnd, deltaTime, bullet_client):
    dorn = bullet_client.getDifferenceQuaternion(ornStart,ornEnd)
    axis,angle = bullet_client.getAxisAngleFromQuaternion(dorn)
    angVel = [(axis[0]*angle)/deltaTime,(axis[1]*angle)/deltaTime,(axis[2]*angle)/deltaTime]
    return angVel
    
  def NormalizeQuaternion(self, orn):
    length2 = orn[0]*orn[0]+orn[1]*orn[1]+orn[2]*orn[2]+orn[3]*orn[3]
    if (length2>0):
    	length = math.sqrt(length2)
    print("Normalize? length=",length)
    
    
  def PostProcessMotionData(self, frameData):
    baseOrn1Start = [frameData[5],frameData[6], frameData[7],frameData[4]]
    self.NormalizeQuaternion(baseOrn1Start)
    chestRotStart = [frameData[9],frameData[10],frameData[11],frameData[8]]
    
    neckRotStart = [frameData[13],frameData[14],frameData[15],frameData[12]]
    rightHipRotStart = [frameData[17],frameData[18],frameData[19],frameData[16]]
    rightAnkleRotStart = [frameData[22],frameData[23],frameData[24],frameData[21]]
    rightShoulderRotStart = [frameData[26],frameData[27],frameData[28],frameData[25]]
    leftHipRotStart = [frameData[31],frameData[32],frameData[33],frameData[30]]
    leftAnkleRotStart = [frameData[36],frameData[37],frameData[38],frameData[35]]
    leftShoulderRotStart = [frameData[40],frameData[41],frameData[42],frameData[39]]
    
  	
  def Slerp(self, frameFraction, frameData, frameDataNext,bullet_client ):
    keyFrameDuration = frameData[0]
    basePos1Start = [frameData[1],frameData[2],frameData[3]]
    basePos1End = [frameDataNext[1],frameDataNext[2],frameDataNext[3]]
    self._basePos = [basePos1Start[0]+frameFraction*(basePos1End[0]-basePos1Start[0]), 
      basePos1Start[1]+frameFraction*(basePos1End[1]-basePos1Start[1]), 
      basePos1Start[2]+frameFraction*(basePos1End[2]-basePos1Start[2])]
    self._baseLinVel = self.ComputeLinVel(basePos1Start,basePos1End, keyFrameDuration)
    baseOrn1Start = [frameData[5],frameData[6], frameData[7],frameData[4]]
    baseOrn1Next = [frameDataNext[5],frameDataNext[6], frameDataNext[7],frameDataNext[4]]
    self._baseOrn = bullet_client.getQuaternionSlerp(baseOrn1Start,baseOrn1Next,frameFraction)
    self._baseAngVel = self.ComputeAngVel(baseOrn1Start,baseOrn1Next, keyFrameDuration, bullet_client)
    
    ##pre-rotate to make z-up
    #y2zPos=[0,0,0.0]
    #y2zOrn = p.getQuaternionFromEuler([1.57,0,0])
    #basePos,baseOrn = p.multiplyTransforms(y2zPos, y2zOrn,basePos1,baseOrn1)

    chestRotStart = [frameData[9],frameData[10],frameData[11],frameData[8]]
    chestRotEnd = [frameDataNext[9],frameDataNext[10],frameDataNext[11],frameDataNext[8]]
    self._chestRot = bullet_client.getQuaternionSlerp(chestRotStart,chestRotEnd,frameFraction)
    self._chestVel = self.ComputeAngVel(chestRotStart,chestRotEnd,keyFrameDuration,bullet_client)
    
    neckRotStart = [frameData[13],frameData[14],frameData[15],frameData[12]]
    neckRotEnd= [frameDataNext[13],frameDataNext[14],frameDataNext[15],frameDataNext[12]]
    self._neckRot =  bullet_client.getQuaternionSlerp(neckRotStart,neckRotEnd,frameFraction)
    self._neckVel = self.ComputeAngVel(neckRotStart,neckRotEnd,keyFrameDuration,bullet_client)
    
    rightHipRotStart = [frameData[17],frameData[18],frameData[19],frameData[16]]
    rightHipRotEnd = [frameDataNext[17],frameDataNext[18],frameDataNext[19],frameDataNext[16]]
    self._rightHipRot = bullet_client.getQuaternionSlerp(rightHipRotStart,rightHipRotEnd,frameFraction)
    self._rightHipVel = self.ComputeAngVel(rightHipRotStart,rightHipRotEnd,keyFrameDuration,bullet_client)
    
    rightKneeRotStart = [frameData[20]]
    rightKneeRotEnd = [frameDataNext[20]]
    self._rightKneeRot = [rightKneeRotStart[0]+frameFraction*(rightKneeRotEnd[0]-rightKneeRotStart[0])]
    self._rightKneeVel = (rightKneeRotEnd[0]-rightKneeRotStart[0])/keyFrameDuration
    
    rightAnkleRotStart = [frameData[22],frameData[23],frameData[24],frameData[21]]
    rightAnkleRotEnd = [frameDataNext[22],frameDataNext[23],frameDataNext[24],frameDataNext[21]]
    self._rightAnkleRot =  bullet_client.getQuaternionSlerp(rightAnkleRotStart,rightAnkleRotEnd,frameFraction)
    self._rightAnkleVel = self.ComputeAngVel(rightAnkleRotStart,rightAnkleRotEnd,keyFrameDuration,bullet_client)
      
    rightShoulderRotStart = [frameData[26],frameData[27],frameData[28],frameData[25]]
    rightShoulderRotEnd = [frameDataNext[26],frameDataNext[27],frameDataNext[28],frameDataNext[25]]
    self._rightShoulderRot = bullet_client.getQuaternionSlerp(rightShoulderRotStart,rightShoulderRotEnd,frameFraction)
    self._rightShoulderVel = self.ComputeAngVel(rightShoulderRotStart,rightShoulderRotEnd, keyFrameDuration,bullet_client)
    
    rightElbowRotStart = [frameData[29]]
    rightElbowRotEnd = [frameDataNext[29]]
    self._rightElbowRot = [rightElbowRotStart[0]+frameFraction*(rightElbowRotEnd[0]-rightElbowRotStart[0])]
    self._rightElbowVel = (rightElbowRotEnd[0]-rightElbowRotStart[0])/keyFrameDuration
    
    leftHipRotStart = [frameData[31],frameData[32],frameData[33],frameData[30]]
    leftHipRotEnd = [frameDataNext[31],frameDataNext[32],frameDataNext[33],frameDataNext[30]]
    self._leftHipRot = bullet_client.getQuaternionSlerp(leftHipRotStart,leftHipRotEnd,frameFraction)
    self._leftHipVel = self.ComputeAngVel(leftHipRotStart, leftHipRotEnd,keyFrameDuration,bullet_client)
    
    leftKneeRotStart = [frameData[34]]
    leftKneeRotEnd = [frameDataNext[34]]
    self._leftKneeRot = [leftKneeRotStart[0] +frameFraction*(leftKneeRotEnd[0]-leftKneeRotStart[0]) ]
    self._leftKneeVel = (leftKneeRotEnd[0]-leftKneeRotStart[0])/keyFrameDuration
    
    leftAnkleRotStart = [frameData[36],frameData[37],frameData[38],frameData[35]]
    leftAnkleRotEnd = [frameDataNext[36],frameDataNext[37],frameDataNext[38],frameDataNext[35]]
    self._leftAnkleRot = bullet_client.getQuaternionSlerp(leftAnkleRotStart,leftAnkleRotEnd,frameFraction)
    self._leftAnkleVel = self.ComputeAngVel(leftAnkleRotStart,leftAnkleRotEnd,keyFrameDuration,bullet_client)

    leftShoulderRotStart = [frameData[40],frameData[41],frameData[42],frameData[39]]
    leftShoulderRotEnd = [frameDataNext[40],frameDataNext[41],frameDataNext[42],frameDataNext[39]]
    self._leftShoulderRot = bullet_client.getQuaternionSlerp(leftShoulderRotStart,leftShoulderRotEnd,frameFraction)
    self._leftShoulderVel = self.ComputeAngVel(leftShoulderRotStart,leftShoulderRotEnd,keyFrameDuration,bullet_client)

    leftElbowRotStart = [frameData[43]]
    leftElbowRotEnd = [frameDataNext[43]]
    self._leftElbowRot = [leftElbowRotStart[0]+frameFraction*(leftElbowRotEnd[0]-leftElbowRotStart[0])]
    self._leftElbowVel = (leftElbowRotEnd[0]-leftElbowRotStart[0])/keyFrameDuration
        

class Humanoid(object):
  def __init__(self, pybullet_client, motion_data, baseShift):
    """Constructs a humanoid and reset it to the initial states.
    Args:
      pybullet_client: The instance of BulletClient to manage different
        simulations.
    """
    self._baseShift = baseShift
    self._pybullet_client = pybullet_client
    self._motion_data = motion_data
    self._humanoid = self._pybullet_client.loadURDF(
      "humanoid/humanoid.urdf", [0,0.9,0],globalScaling=0.25)
    
    pose = HumanoidPose()
    
    for i in range (self._motion_data.NumFrames()-1):
      frameData = self._motion_data._motion_data['Frames'][i]
      pose.PostProcessMotionData(frameData)
    
    self._pybullet_client.resetBasePositionAndOrientation(self._humanoid,self._baseShift,[0,0,0,1])
    self._pybullet_client.changeDynamics(self._humanoid, -1, linearDamping=0, angularDamping=0)
    for j in range (self._pybullet_client.getNumJoints(self._humanoid)):
      self._pybullet_client.changeDynamics(self._humanoid, j, linearDamping=0, angularDamping=0)
      self._pybullet_client.changeVisualShape(self._humanoid, j , rgbaColor=[1,1,1,1])
    
    self._initial_state = self._pybullet_client.saveState()
    self._allowed_body_parts=[11,14]
    self.Reset()
    
  def Reset(self):
    self._pybullet_client.restoreState(self._initial_state)
    self.SetFrameTime(0)
    pose = self.InitializePoseFromMotionData()
    self.ApplyPose(pose, True)


  def SetFrameTime(self, t):
    self._frameTime = t
    self._frame = int(self._frameTime)
    self._frameNext = self._frame+1
    if (self._frameNext >=  self._motion_data.NumFrames()):
      self._frameNext = self._frame
    self._frameFraction = self._frameTime - self._frame

  def Terminates(self):
    #check if any non-allowed body part hits the ground
    terminates=False
    pts = self._pybullet_client.getContactPoints()
    for p in pts:
      part = -1
      if (p[1]==self._humanoid):
        part=p[3]
      if (p[2]==self._humanoid):
        part=p[4]
      if (part not in self._allowed_body_parts):
        terminates=True
      
    return terminates
        
        
      
    
    
  def InitializePoseFromMotionData(self):
    frameData = self._motion_data._motion_data['Frames'][self._frame]
    frameDataNext = self._motion_data._motion_data['Frames'][self._frameNext]
    pose = HumanoidPose()
    pose.Slerp(self._frameFraction, frameData, frameDataNext, self._pybullet_client)
    return pose
    

  def ApplyAction(self, action):
    #turn action into pose
    pose = HumanoidPose()
    #todo: convert action vector into pose
    #convert from angle-axis to quaternion for spherical joints
    
    initializeBase = False
    self.ApplyPose(pose, initializeBase)
    
  def ApplyPose(self, pose, initializeBase):
    if (initializeBase):
      self._pybullet_client.changeVisualShape(self._humanoid, 2 , rgbaColor=[1,0,0,1])
      basePos=[pose._basePos[0]+self._baseShift[0],pose._basePos[1]+self._baseShift[1],pose._basePos[2]+self._baseShift[2]]
      self._pybullet_client.resetBasePositionAndOrientation(self._humanoid,
        basePos, pose._baseOrn)
      self._pybullet_client.resetBaseVelocity(self._humanoid, pose._baseLinVel, pose._baseAngVel)
      print("resetBaseVelocity=",pose._baseLinVel)
    else:
      self._pybullet_client.changeVisualShape(self._humanoid, 2 , rgbaColor=[1,1,1,1])
    maxForce=1000
    kp=0.8
    chest=1
    neck=2
    rightShoulder=3
    rightElbow=4
    leftShoulder=6
    leftElbow = 7
    rightHip = 9
    rightKnee=10
    rightAnkle=11
    leftHip = 12
    leftKnee=13
    leftAnkle=14
    controlMode = self._pybullet_client.POSITION_CONTROL
    
    if (initializeBase):
      self._pybullet_client.resetJointStateMultiDof(self._humanoid,chest,pose._chestRot)
      self._pybullet_client.resetJointStateMultiDof(self._humanoid,neck,pose._neckRot)
      self._pybullet_client.resetJointStateMultiDof(self._humanoid,rightHip,pose._rightHipRot)
      self._pybullet_client.resetJointStateMultiDof(self._humanoid,rightKnee,pose._rightKneeRot)
      self._pybullet_client.resetJointStateMultiDof(self._humanoid,rightAnkle,pose._rightAnkleRot)
      self._pybullet_client.resetJointStateMultiDof(self._humanoid,rightShoulder,pose._rightShoulderRot)
      self._pybullet_client.resetJointStateMultiDof(self._humanoid,rightElbow, pose._rightElbowRot)
      self._pybullet_client.resetJointStateMultiDof(self._humanoid,leftHip, pose._leftHipRot)
      self._pybullet_client.resetJointStateMultiDof(self._humanoid,leftKnee, pose._leftKneeRot)
      self._pybullet_client.resetJointStateMultiDof(self._humanoid,leftAnkle, pose._leftAnkleRot)
      self._pybullet_client.resetJointStateMultiDof(self._humanoid,leftShoulder, pose._leftShoulderRot)
      self._pybullet_client.resetJointStateMultiDof(self._humanoid,leftElbow, pose._leftElbowRot)
    
    self._pybullet_client.setJointMotorControlMultiDof(self._humanoid,chest,controlMode, targetPosition=pose._chestRot,positionGain=kp, force=maxForce)
    self._pybullet_client.setJointMotorControlMultiDof(self._humanoid,neck,controlMode,targetPosition=pose._neckRot,positionGain=kp, force=maxForce)
    self._pybullet_client.setJointMotorControlMultiDof(self._humanoid,rightHip,controlMode,targetPosition=pose._rightHipRot,positionGain=kp, force=maxForce)
    self._pybullet_client.setJointMotorControlMultiDof(self._humanoid,rightKnee,controlMode,targetPosition=pose._rightKneeRot,positionGain=kp, force=maxForce)
    self._pybullet_client.setJointMotorControlMultiDof(self._humanoid,rightAnkle,controlMode,targetPosition=pose._rightAnkleRot,positionGain=kp, force=maxForce)
    self._pybullet_client.setJointMotorControlMultiDof(self._humanoid,rightShoulder,controlMode,targetPosition=pose._rightShoulderRot,positionGain=kp, force=maxForce)
    self._pybullet_client.setJointMotorControlMultiDof(self._humanoid,rightElbow, controlMode,targetPosition=pose._rightElbowRot,positionGain=kp, force=maxForce)
    self._pybullet_client.setJointMotorControlMultiDof(self._humanoid,leftHip, controlMode,targetPosition=pose._leftHipRot,positionGain=kp, force=maxForce)
    self._pybullet_client.setJointMotorControlMultiDof(self._humanoid,leftKnee, controlMode,targetPosition=pose._leftKneeRot,positionGain=kp, force=maxForce)
    self._pybullet_client.setJointMotorControlMultiDof(self._humanoid,leftAnkle, controlMode,targetPosition=pose._leftAnkleRot,positionGain=kp, force=maxForce)
    self._pybullet_client.setJointMotorControlMultiDof(self._humanoid,leftShoulder, controlMode,targetPosition=pose._leftShoulderRot,positionGain=kp, force=maxForce)
    self._pybullet_client.setJointMotorControlMultiDof(self._humanoid,leftElbow, controlMode,targetPosition=pose._leftElbowRot,positionGain=kp, force=maxForce)
    