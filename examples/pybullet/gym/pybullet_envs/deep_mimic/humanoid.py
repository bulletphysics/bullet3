import os,  inspect
import math
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)

from pybullet_utils.bullet_client import BulletClient
import pybullet_data

jointTypes = ["JOINT_REVOLUTE","JOINT_PRISMATIC",
              "JOINT_SPHERICAL","JOINT_PLANAR","JOINT_FIXED"]
              
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
    #print("Normalize? length=",length)

    
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
    self._rightKneeVel = [(rightKneeRotEnd[0]-rightKneeRotStart[0])/keyFrameDuration]
    
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
    self._rightElbowVel = [(rightElbowRotEnd[0]-rightElbowRotStart[0])/keyFrameDuration]
    
    leftHipRotStart = [frameData[31],frameData[32],frameData[33],frameData[30]]
    leftHipRotEnd = [frameDataNext[31],frameDataNext[32],frameDataNext[33],frameDataNext[30]]
    self._leftHipRot = bullet_client.getQuaternionSlerp(leftHipRotStart,leftHipRotEnd,frameFraction)
    self._leftHipVel = self.ComputeAngVel(leftHipRotStart, leftHipRotEnd,keyFrameDuration,bullet_client)
    
    leftKneeRotStart = [frameData[34]]
    leftKneeRotEnd = [frameDataNext[34]]
    self._leftKneeRot = [leftKneeRotStart[0] +frameFraction*(leftKneeRotEnd[0]-leftKneeRotStart[0]) ]
    self._leftKneeVel = [(leftKneeRotEnd[0]-leftKneeRotStart[0])/keyFrameDuration]
    
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
    self._leftElbowVel = [(leftElbowRotEnd[0]-leftElbowRotStart[0])/keyFrameDuration]
        

class Humanoid(object):
  def __init__(self, pybullet_client, motion_data, baseShift):
    """Constructs a humanoid and reset it to the initial states.
    Args:
      pybullet_client: The instance of BulletClient to manage different
        simulations.
    """
    self._baseShift = baseShift
    self._pybullet_client = pybullet_client
    
    self.kin_client = BulletClient(pybullet_client.DIRECT)# use SHARED_MEMORY for visual debugging, start a GUI physics server first
    self.kin_client.resetSimulation()
    self.kin_client.setAdditionalSearchPath(pybullet_data.getDataPath())
    self.kin_client.configureDebugVisualizer(self.kin_client.COV_ENABLE_Y_AXIS_UP,1)
    self.kin_client.setGravity(0,-9.8,0)
    
    self._motion_data = motion_data
    print("LOADING humanoid!")
    self._humanoid = self._pybullet_client.loadURDF(
      "humanoid/humanoid.urdf", [0,0.9,0],globalScaling=0.25, useFixedBase=False)
      
    self._kinematicHumanoid = self.kin_client.loadURDF(
      "humanoid/humanoid.urdf", [0,0.9,0],globalScaling=0.25, useFixedBase=False)
      
      
    #print("human #joints=", self._pybullet_client.getNumJoints(self._humanoid))
    pose = HumanoidPose()
    
    for i in range (self._motion_data.NumFrames()-1):
      frameData = self._motion_data._motion_data['Frames'][i]
      pose.PostProcessMotionData(frameData)
    
    self._pybullet_client.resetBasePositionAndOrientation(self._humanoid,self._baseShift,[0,0,0,1])
    self._pybullet_client.changeDynamics(self._humanoid, -1, linearDamping=0, angularDamping=0)
    for j in range (self._pybullet_client.getNumJoints(self._humanoid)):
      ji = self._pybullet_client.getJointInfo(self._humanoid,j)
      self._pybullet_client.changeDynamics(self._humanoid, j, linearDamping=0, angularDamping=0)
      self._pybullet_client.changeVisualShape(self._humanoid, j , rgbaColor=[1,1,1,1])
      #print("joint[",j,"].type=",jointTypes[ji[2]])
      #print("joint[",j,"].name=",ji[1])
    
    self._initial_state = self._pybullet_client.saveState()
    self._allowed_body_parts=[11,14]
    self.Reset()
    
  def Reset(self):
    self._pybullet_client.restoreState(self._initial_state)
    self.SetSimTime(0)
    pose = self.InitializePoseFromMotionData()
    self.ApplyPose(pose, True, True, self._humanoid, self._pybullet_client)

  def RenderReference(self, t):
    self.SetSimTime(t)
    frameData = self._motion_data._motion_data['Frames'][self._frame]
    frameDataNext = self._motion_data._motion_data['Frames'][self._frameNext]
    pose = HumanoidPose()
    pose.Slerp(self._frameFraction, frameData, frameDataNext, self._pybullet_client)
    self.ApplyPose(pose, True, True, self._humanoid, self._pybullet_client)

  def CalcCycleCount(self, simTime, cycleTime):
    phases = simTime / cycleTime;
    count = math.floor(phases)
    loop = True
    #count = (loop) ? count : cMathUtil::Clamp(count, 0, 1);
    return count

  def SetSimTime(self, t):
    self._simTime = t
    #print("SetTimeTime time =",t)
    keyFrameDuration = self._motion_data.KeyFrameDuraction()
    cycleTime = keyFrameDuration*(self._motion_data.NumFrames()-1)
    #print("self._motion_data.NumFrames()=",self._motion_data.NumFrames())
    #print("cycleTime=",cycleTime)
    cycles = self.CalcCycleCount(t, cycleTime)
    #print("cycles=",cycles)
    frameTime = t - cycles*cycleTime
    if (frameTime<0):
      frameTime += cycleTime
    
    #print("keyFrameDuration=",keyFrameDuration)  
    #print("frameTime=",frameTime)
    self._frame = int(frameTime/keyFrameDuration)
    #print("self._frame=",self._frame)
    
    self._frameNext = self._frame+1
    if (self._frameNext >=  self._motion_data.NumFrames()):
      self._frameNext = self._frame

    self._frameFraction = (frameTime - self._frame*keyFrameDuration)/(keyFrameDuration)
    #print("self._frameFraction=",self._frameFraction)

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
      if (part >=0 and part not in self._allowed_body_parts):
        terminates=True
      
    return terminates
        
  def BuildHeadingTrans(self, rootOrn):
    #align root transform 'forward' with world-space x axis
    eul = self._pybullet_client.getEulerFromQuaternion(rootOrn)
    refDir = [1,0,0]
    rotVec = self._pybullet_client.rotateVector(rootOrn, refDir)
    heading = math.atan2(-rotVec[2], rotVec[0])
    heading2=eul[1]
    #print("heading=",heading)
    headingOrn = self._pybullet_client.getQuaternionFromAxisAngle([0,1,0],-heading)
    return headingOrn

  def GetPhase(self):
    keyFrameDuration = self._motion_data.KeyFrameDuraction()
    cycleTime = keyFrameDuration*(self._motion_data.NumFrames()-1)
    phase = self._simTime / cycleTime
    phase = math.fmod(phase,1.0)
    if (phase<0):
      phase += 1
    return phase

  def BuildOriginTrans(self):
    rootPos,rootOrn = self._pybullet_client.getBasePositionAndOrientation(self._humanoid)
    
    #print("rootPos=",rootPos, " rootOrn=",rootOrn)
    invRootPos=[-rootPos[0], 0, -rootPos[2]]
    #invOrigTransPos, invOrigTransOrn = self._pybullet_client.invertTransform(rootPos,rootOrn)
    headingOrn = self.BuildHeadingTrans(rootOrn)
    #print("headingOrn=",headingOrn)
    headingMat = self._pybullet_client.getMatrixFromQuaternion(headingOrn)
    #print("headingMat=",headingMat)
    #dummy, rootOrnWithoutHeading = self._pybullet_client.multiplyTransforms([0,0,0],headingOrn, [0,0,0], rootOrn)
    #dummy, invOrigTransOrn = self._pybullet_client.multiplyTransforms([0,0,0],rootOrnWithoutHeading, invOrigTransPos, invOrigTransOrn)
    
    invOrigTransPos, invOrigTransOrn = self._pybullet_client.multiplyTransforms( [0,0,0],headingOrn, invRootPos,[0,0,0,1])
    #print("invOrigTransPos=",invOrigTransPos)
    #print("invOrigTransOrn=",invOrigTransOrn)
    invOrigTransMat = self._pybullet_client.getMatrixFromQuaternion(invOrigTransOrn)
    #print("invOrigTransMat =",invOrigTransMat )
    return invOrigTransPos, invOrigTransOrn
    
  def InitializePoseFromMotionData(self):
    frameData = self._motion_data._motion_data['Frames'][self._frame]
    frameDataNext = self._motion_data._motion_data['Frames'][self._frameNext]
    pose = HumanoidPose()
    pose.Slerp(self._frameFraction, frameData, frameDataNext, self._pybullet_client)
    return pose
    

  
    
  def ApplyAction(self, action):
    #turn action into pose
    pose = HumanoidPose()
    pose.Reset()
    index=0
    angle = action[index]
    axis = [action[index+1],action[index+2],action[index+3]]
    index+=4
    pose._chestRot = self._pybullet_client.getQuaternionFromAxisAngle(axis,angle)
    #print("pose._chestRot=",pose._chestRot)

    angle = action[index]
    axis = [action[index+1],action[index+2],action[index+3]]
    index+=4
    pose._neckRot = self._pybullet_client.getQuaternionFromAxisAngle(axis,angle)
    
    angle = action[index]
    axis = [action[index+1],action[index+2],action[index+3]]
    index+=4
    pose._rightHipRot = self._pybullet_client.getQuaternionFromAxisAngle(axis,angle)
    
    angle = action[index]
    index+=1
    pose._rightKneeRot = [angle]
        
    angle = action[index]
    axis = [action[index+1],action[index+2],action[index+3]]
    index+=4
    pose._rightAnkleRot = self._pybullet_client.getQuaternionFromAxisAngle(axis,angle)
    
    angle = action[index]
    axis = [action[index+1],action[index+2],action[index+3]]
    index+=4
    pose._rightShoulderRot = self._pybullet_client.getQuaternionFromAxisAngle(axis,angle)
    
    angle = action[index]
    index+=1
    pose._rightElbowRot = [angle]
    
    angle = action[index]
    axis = [action[index+1],action[index+2],action[index+3]]
    index+=4
    pose._leftHipRot = self._pybullet_client.getQuaternionFromAxisAngle(axis,angle)
    
    angle = action[index]
    index+=1
    pose._leftKneeRot = [angle]
    
    
    angle = action[index]
    axis = [action[index+1],action[index+2],action[index+3]]
    index+=4
    pose._leftAnkleRot = self._pybullet_client.getQuaternionFromAxisAngle(axis,angle)
    
    angle = action[index]
    axis = [action[index+1],action[index+2],action[index+3]]
    index+=4
    pose._leftShoulderRot = self._pybullet_client.getQuaternionFromAxisAngle(axis,angle)
        
    angle = action[index]
    index+=1
    pose._leftElbowRot = [angle]
    
    
    #print("index=",index)
    
    initializeBase = False
    initializeVelocities = False
    self.ApplyPose(pose, initializeBase, initializeVelocities, self._humanoid, self._pybullet_client)
    
    
  def ApplyPose(self, pose, initializeBase, initializeVelocities, humanoid,bc):
    #todo: get tunable parametes from a json file or from URDF (kd, maxForce)
    if (initializeBase):
      bc.changeVisualShape(humanoid, 2 , rgbaColor=[1,0,0,1])
      basePos=[pose._basePos[0]+self._baseShift[0],pose._basePos[1]+self._baseShift[1],pose._basePos[2]+self._baseShift[2]]
      
      bc.resetBasePositionAndOrientation(humanoid,
        basePos, pose._baseOrn)
      if initializeVelocities:
        bc.resetBaseVelocity(humanoid, pose._baseLinVel, pose._baseAngVel)
        #print("resetBaseVelocity=",pose._baseLinVel)
    else:
      bc.changeVisualShape(humanoid, 2 , rgbaColor=[1,1,1,1])
    
    
    
    kp=0.03
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
    controlMode = bc.POSITION_CONTROL
    
    if (initializeBase):
      if initializeVelocities:
        bc.resetJointStateMultiDof(humanoid,chest,pose._chestRot, pose._chestVel)
        bc.resetJointStateMultiDof(humanoid,neck,pose._neckRot, pose._neckVel)
        bc.resetJointStateMultiDof(humanoid,rightHip,pose._rightHipRot, pose._rightHipVel)
        bc.resetJointStateMultiDof(humanoid,rightKnee,pose._rightKneeRot, pose._rightKneeVel)
        bc.resetJointStateMultiDof(humanoid,rightAnkle,pose._rightAnkleRot, pose._rightAnkleVel)
        bc.resetJointStateMultiDof(humanoid,rightShoulder,pose._rightShoulderRot, pose._rightShoulderVel)
        bc.resetJointStateMultiDof(humanoid,rightElbow, pose._rightElbowRot, pose._rightElbowVel)
        bc.resetJointStateMultiDof(humanoid,leftHip, pose._leftHipRot, pose._leftHipVel)
        bc.resetJointStateMultiDof(humanoid,leftKnee, pose._leftKneeRot, pose._leftKneeVel)
        bc.resetJointStateMultiDof(humanoid,leftAnkle, pose._leftAnkleRot, pose._leftAnkleVel)
        bc.resetJointStateMultiDof(humanoid,leftShoulder, pose._leftShoulderRot, pose._leftShoulderVel)
        bc.resetJointStateMultiDof(humanoid,leftElbow, pose._leftElbowRot, pose._leftElbowVel)
      else:
        bc.resetJointStateMultiDof(humanoid,chest,pose._chestRot)
        bc.resetJointStateMultiDof(humanoid,neck,pose._neckRot)
        bc.resetJointStateMultiDof(humanoid,rightHip,pose._rightHipRot)
        bc.resetJointStateMultiDof(humanoid,rightKnee,pose._rightKneeRot)
        bc.resetJointStateMultiDof(humanoid,rightAnkle,pose._rightAnkleRot)
        bc.resetJointStateMultiDof(humanoid,rightShoulder,pose._rightShoulderRot)
        bc.resetJointStateMultiDof(humanoid,rightElbow, pose._rightElbowRot)
        bc.resetJointStateMultiDof(humanoid,leftHip, pose._leftHipRot)
        bc.resetJointStateMultiDof(humanoid,leftKnee, pose._leftKneeRot)
        bc.resetJointStateMultiDof(humanoid,leftAnkle, pose._leftAnkleRot)
        bc.resetJointStateMultiDof(humanoid,leftShoulder, pose._leftShoulderRot)
        bc.resetJointStateMultiDof(humanoid,leftElbow, pose._leftElbowRot)
    
    bc.setJointMotorControlMultiDof(humanoid,chest,controlMode, targetPosition=pose._chestRot,positionGain=kp, force=[200])
    bc.setJointMotorControlMultiDof(humanoid,neck,controlMode,targetPosition=pose._neckRot,positionGain=kp, force=[50])
    bc.setJointMotorControlMultiDof(humanoid,rightHip,controlMode,targetPosition=pose._rightHipRot,positionGain=kp, force=[200])
    bc.setJointMotorControlMultiDof(humanoid,rightKnee,controlMode,targetPosition=pose._rightKneeRot,positionGain=kp, force=[150])
    bc.setJointMotorControlMultiDof(humanoid,rightAnkle,controlMode,targetPosition=pose._rightAnkleRot,positionGain=kp, force=[90])
    bc.setJointMotorControlMultiDof(humanoid,rightShoulder,controlMode,targetPosition=pose._rightShoulderRot,positionGain=kp, force=[100])
    bc.setJointMotorControlMultiDof(humanoid,rightElbow, controlMode,targetPosition=pose._rightElbowRot,positionGain=kp, force=[60])
    bc.setJointMotorControlMultiDof(humanoid,leftHip, controlMode,targetPosition=pose._leftHipRot,positionGain=kp, force=[200])
    bc.setJointMotorControlMultiDof(humanoid,leftKnee, controlMode,targetPosition=pose._leftKneeRot,positionGain=kp, force=[150])
    bc.setJointMotorControlMultiDof(humanoid,leftAnkle, controlMode,targetPosition=pose._leftAnkleRot,positionGain=kp, force=[90])
    bc.setJointMotorControlMultiDof(humanoid,leftShoulder, controlMode,targetPosition=pose._leftShoulderRot,positionGain=kp, force=[100])
    bc.setJointMotorControlMultiDof(humanoid,leftElbow, controlMode,targetPosition=pose._leftElbowRot,positionGain=kp, force=[60])

    #debug space
    #if (False):
    #  for j in range (bc.getNumJoints(self._humanoid)):
    #    js = bc.getJointState(self._humanoid, j)
    #    bc.resetJointState(self._humanoidDebug, j,js[0])
    #    jsm = bc.getJointStateMultiDof(self._humanoid, j)
    #    if (len(jsm[0])>0):
    #      bc.resetJointStateMultiDof(self._humanoidDebug,j,jsm[0])
        
  def GetState(self):

    stateVector = []
    phase = self.GetPhase()
    #print("phase=",phase)
    stateVector.append(phase)
    
    rootTransPos, rootTransOrn=self.BuildOriginTrans()
    basePos,baseOrn = self._pybullet_client.getBasePositionAndOrientation(self._humanoid)
    
    rootPosRel, dummy = self._pybullet_client.multiplyTransforms(rootTransPos, rootTransOrn, basePos,[0,0,0,1])
    #print("!!!rootPosRel =",rootPosRel )
    #print("rootTransPos=",rootTransPos)
    #print("basePos=",basePos)
    localPos,localOrn = self._pybullet_client.multiplyTransforms( rootTransPos, rootTransOrn , basePos,baseOrn )
    
    localPos=[localPos[0]-rootPosRel[0],localPos[1]-rootPosRel[1],localPos[2]-rootPosRel[2]]
    #print("localPos=",localPos)
        
    stateVector.append(rootPosRel[1])
    
    self.pb2dmJoints=[0,1,2,9,10,11,3,4,5,12,13,14,6,7,8]
    
    for pbJoint in range (self._pybullet_client.getNumJoints(self._humanoid)):
      j = self.pb2dmJoints[pbJoint]
      #print("joint order:",j)
      ls = self._pybullet_client.getLinkState(self._humanoid, j, computeForwardKinematics=True)
      linkPos = ls[0]
      linkOrn = ls[1]
      linkPosLocal, linkOrnLocal = self._pybullet_client.multiplyTransforms(rootTransPos, rootTransOrn, linkPos,linkOrn)
      if (linkOrnLocal[3]<0):
        linkOrnLocal=[-linkOrnLocal[0],-linkOrnLocal[1],-linkOrnLocal[2],-linkOrnLocal[3]]
      linkPosLocal=[linkPosLocal[0]-rootPosRel[0],linkPosLocal[1]-rootPosRel[1],linkPosLocal[2]-rootPosRel[2]]
      for l in linkPosLocal:
        stateVector.append(l)
      
      #re-order the quaternion, DeepMimic uses w,x,y,z
      stateVector.append(linkOrnLocal[3])
      stateVector.append(linkOrnLocal[0])
      stateVector.append(linkOrnLocal[1])
      stateVector.append(linkOrnLocal[2])
    
    
    for pbJoint in range (self._pybullet_client.getNumJoints(self._humanoid)):
      j = self.pb2dmJoints[pbJoint]
      ls = self._pybullet_client.getLinkState(self._humanoid, j, computeLinkVelocity=True)
      linkLinVel = ls[6]
      linkAngVel = ls[7]
      for l in linkLinVel:
        stateVector.append(l)
      for l in linkAngVel:
        stateVector.append(l)
    
    #print("stateVector len=",len(stateVector))  
    #for st in range (len(stateVector)):
    #  print("state[",st,"]=",stateVector[st])
    return stateVector
  
  
  def GetReward(self):
    #from DeepMimic double cSceneImitate::CalcRewardImitate
    pose_w = 0.5
    vel_w = 0.05
    end_eff_w = 0 #0.15
    root_w = 0 #0.2
    com_w = 0.1

    total_w = pose_w + vel_w + end_eff_w + root_w + com_w
    pose_w /= total_w
    vel_w /= total_w
    end_eff_w /= total_w
    root_w /= total_w
    com_w /= total_w

    pose_scale = 2
    vel_scale = 0.1
    end_eff_scale = 40
    root_scale = 5
    com_scale = 10
    err_scale = 1

    reward = 0

    pose_err = 0
    vel_err = 0
    end_eff_err = 0
    root_err = 0
    com_err = 0
    heading_err = 0
    
    #create a mimic reward, comparing the dynamics humanoid with a kinematic one
    
    pose = self.InitializePoseFromMotionData()
    #print("self._kinematicHumanoid=",self._kinematicHumanoid)
    #print("kinematicHumanoid #joints=",self.kin_client.getNumJoints(self._kinematicHumanoid))
    self.ApplyPose(pose, True, True, self._kinematicHumanoid, self.kin_client)

    #const Eigen::VectorXd& pose0 = sim_char.GetPose();
    #const Eigen::VectorXd& vel0 = sim_char.GetVel();
    #const Eigen::VectorXd& pose1 = kin_char.GetPose();
    #const Eigen::VectorXd& vel1 = kin_char.GetVel();
    #tMatrix origin_trans = sim_char.BuildOriginTrans();
    #tMatrix kin_origin_trans = kin_char.BuildOriginTrans();
    #
    #tVector com0_world = sim_char.CalcCOM();
    #tVector com_vel0_world = sim_char.CalcCOMVel();
    #tVector com1_world;
    #tVector com_vel1_world;
    #cRBDUtil::CalcCoM(joint_mat, body_defs, pose1, vel1, com1_world, com_vel1_world);
    #
    root_id = 0
    #tVector root_pos0 = cKinTree::GetRootPos(joint_mat, pose0);
    #tVector root_pos1 = cKinTree::GetRootPos(joint_mat, pose1);
    #tQuaternion root_rot0 = cKinTree::GetRootRot(joint_mat, pose0);
    #tQuaternion root_rot1 = cKinTree::GetRootRot(joint_mat, pose1);
    #tVector root_vel0 = cKinTree::GetRootVel(joint_mat, vel0);
    #tVector root_vel1 = cKinTree::GetRootVel(joint_mat, vel1);
    #tVector root_ang_vel0 = cKinTree::GetRootAngVel(joint_mat, vel0);
    #tVector root_ang_vel1 = cKinTree::GetRootAngVel(joint_mat, vel1);

    mJointWeights = [0.20833,0.10416, 0.0625, 0.10416,
      0.0625, 0.041666666666666671, 0.0625, 0.0416,
      0.00, 0.10416,  0.0625, 0.0416, 0.0625, 0.0416, 0.0000]
      
    num_end_effs = 0
    num_joints = 15
    
    root_rot_w = mJointWeights[root_id]
    #pose_err += root_rot_w * cKinTree::CalcRootRotErr(joint_mat, pose0, pose1)
    #vel_err += root_rot_w * cKinTree::CalcRootAngVelErr(joint_mat, vel0, vel1)

    for j in range (num_joints):
      curr_pose_err = 0
      curr_vel_err = 0
      w = mJointWeights[j];
      
      simJointInfo = self._pybullet_client.getJointStateMultiDof(self._humanoid, j)
      
      #print("simJointInfo.pos=",simJointInfo[0])
      #print("simJointInfo.vel=",simJointInfo[1])
      kinJointInfo = self.kin_client.getJointStateMultiDof(self._kinematicHumanoid,j)
      #print("kinJointInfo.pos=",kinJointInfo[0])
      #print("kinJointInfo.vel=",kinJointInfo[1])
      if (len(simJointInfo[0])==1):
        angle = simJointInfo[0][0]-kinJointInfo[0][0]
        curr_pose_err = angle*angle
        velDiff = simJointInfo[1][0]-kinJointInfo[1][0]
        curr_vel_err = velDiff*velDiff
      if (len(simJointInfo[0])==4):
        #print("quaternion diff")
        diffQuat = self._pybullet_client.getDifferenceQuaternion(simJointInfo[0],kinJointInfo[0])
        axis,angle = self._pybullet_client.getAxisAngleFromQuaternion(diffQuat)
        curr_pose_err = angle*angle
        diffVel = [simJointInfo[1][0]-kinJointInfo[1][0],simJointInfo[1][1]-kinJointInfo[1][1],simJointInfo[1][2]-kinJointInfo[1][2]]
        curr_vel_err = diffVel[0]*diffVel[0]+diffVel[1]*diffVel[1]+diffVel[2]*diffVel[2]
      
      
      pose_err += w * curr_pose_err
      vel_err += w * curr_vel_err
    
    #  bool is_end_eff = sim_char.IsEndEffector(j)
    #  if (is_end_eff)
    #  {
    #    tVector pos0 = sim_char.CalcJointPos(j)
    #    tVector pos1 = cKinTree::CalcJointWorldPos(joint_mat, pose1, j)
    #    double ground_h0 = mGround->SampleHeight(pos0)
    #    double ground_h1 = kin_char.GetOriginPos()[1]
    #
    #    tVector pos_rel0 = pos0 - root_pos0
    #    tVector pos_rel1 = pos1 - root_pos1
    #    pos_rel0[1] = pos0[1] - ground_h0
    #    pos_rel1[1] = pos1[1] - ground_h1
    #
    #    pos_rel0 = origin_trans * pos_rel0
    #    pos_rel1 = kin_origin_trans * pos_rel1
    #
    #    curr_end_err = (pos_rel1 - pos_rel0).squaredNorm()
    #    end_eff_err += curr_end_err;
    #    ++num_end_effs;
    #  }
    #}
    #if (num_end_effs > 0):
    #  end_eff_err /= num_end_effs
    #
    #double root_ground_h0 = mGround->SampleHeight(sim_char.GetRootPos())
    #double root_ground_h1 = kin_char.GetOriginPos()[1]
    #root_pos0[1] -= root_ground_h0
    #root_pos1[1] -= root_ground_h1
    #root_pos_err = (root_pos0 - root_pos1).squaredNorm()
    #  
    #root_rot_err = cMathUtil::QuatDiffTheta(root_rot0, root_rot1)
    #root_rot_err *= root_rot_err

    #root_vel_err = (root_vel1 - root_vel0).squaredNorm()
    #root_ang_vel_err = (root_ang_vel1 - root_ang_vel0).squaredNorm()

    #root_err = root_pos_err
    #    + 0.1 * root_rot_err
    #    + 0.01 * root_vel_err
    #    + 0.001 * root_ang_vel_err
    #com_err = 0.1 * (com_vel1_world - com_vel0_world).squaredNorm()
    
    #print("pose_err=",pose_err)
    #print("vel_err=",vel_err)
    pose_reward = math.exp(-err_scale * pose_scale * pose_err)
    vel_reward = math.exp(-err_scale * vel_scale * vel_err)
    end_eff_reward = math.exp(-err_scale * end_eff_scale * end_eff_err)
    root_reward = math.exp(-err_scale * root_scale * root_err)
    com_reward = math.exp(-err_scale * com_scale * com_err)

    reward = pose_w * pose_reward + vel_w * vel_reward + end_eff_w * end_eff_reward + root_w * root_reward + com_w * com_reward

    #print("reward = %f (pose_reward=%f, vel_reward=%f, end_eff_reward=%f, root_reward=%f, com_reward=%f)\n", reward,
    # pose_reward,vel_reward,end_eff_reward, root_reward, com_reward);
    
    return reward

  def GetBasePosition(self):
    pos,orn = self._pybullet_client.getBasePositionAndOrientation(self._humanoid)
    return pos

