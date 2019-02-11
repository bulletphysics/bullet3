
from pybullet_utils import pd_controller_stable 
from pybullet_envs.deep_mimic.env import humanoid_pose_interpolator
import math

chest=1
neck=2
rightHip=3
rightKnee=4
rightAnkle=5
rightShoulder=6
rightElbow=7
leftHip=9
leftKnee=10
leftAnkle=11
leftShoulder=12
leftElbow=13
jointFrictionForce = 0

class HumanoidStablePD(object):
  def __init__(self, pybullet_client, mocap_data, timeStep, useFixedBase=True):
    self._pybullet_client = pybullet_client
    self._mocap_data = mocap_data
    print("LOADING humanoid!")
    
    self._sim_model = self._pybullet_client.loadURDF(
      "humanoid/humanoid.urdf", [0,0.889540259,0],globalScaling=0.25, useFixedBase=useFixedBase, flags=self._pybullet_client.URDF_MAINTAIN_LINK_ORDER)

    self._kin_model = self._pybullet_client.loadURDF(
      "humanoid/humanoid.urdf", [0,12.85,0],globalScaling=0.25, useFixedBase=True, flags=self._pybullet_client.URDF_MAINTAIN_LINK_ORDER)
    
    self._pybullet_client.changeDynamics(self._sim_model, -1, lateralFriction=0.9)
    for j in range (self._pybullet_client.getNumJoints(self._sim_model)):
      self._pybullet_client.changeDynamics(self._sim_model, j, lateralFriction=0.9)
      
    self._pybullet_client.changeDynamics(self._sim_model, -1, linearDamping=0, angularDamping=0)
    self._pybullet_client.changeDynamics(self._kin_model, -1, linearDamping=0, angularDamping=0)
    self._poseInterpolator = humanoid_pose_interpolator.HumanoidPoseInterpolator()
    
    for i in range (self._mocap_data.NumFrames()-1):
      frameData = self._mocap_data._motion_data['Frames'][i]
      self._poseInterpolator.PostProcessMotionData(frameData)
      
    self._stablePD = pd_controller_stable.PDControllerStableMultiDof(self._pybullet_client)
    self._timeStep = timeStep
    self._kpOrg = [0,0,0,0,0,0,0,1000,1000,1000,1000,100,100,100,100,500,500,500,500,500,400,400,400,400,400,400,400,400,300,500,500,500,500,500,400,400,400,400,400,400,400,400,300]
    self._kdOrg = [0,0,0,0,0,0,0,100,100,100,100,10,10,10,10,50,50,50,50,50,40,40,40,40,40,40,40,40,30,50,50,50,50,50,40,40,40,40,40,40,40,40,30]
   
    self._jointIndicesAll = [chest,neck, rightHip,rightKnee,rightAnkle,rightShoulder,rightElbow,leftHip,leftKnee,leftAnkle,leftShoulder,leftElbow]
    for j in self._jointIndicesAll:
      #self._pybullet_client.setJointMotorControlMultiDof(self._sim_model, j, self._pybullet_client.POSITION_CONTROL, force=[1,1,1])
      self._pybullet_client.setJointMotorControl2(self._sim_model, j, self._pybullet_client.POSITION_CONTROL, targetPosition=0, positionGain=0, targetVelocity=0,force=jointFrictionForce)
      self._pybullet_client.setJointMotorControlMultiDof(self._sim_model, j, self._pybullet_client.POSITION_CONTROL,targetPosition=[0,0,0,1], targetVelocity=[0,0,0], positionGain=0,velocityGain=1,force=[jointFrictionForce,jointFrictionForce,jointFrictionForce])
      self._pybullet_client.setJointMotorControl2(self._kin_model, j, self._pybullet_client.POSITION_CONTROL, targetPosition=0, positionGain=0, targetVelocity=0,force=0)
      self._pybullet_client.setJointMotorControlMultiDof(self._kin_model, j, self._pybullet_client.POSITION_CONTROL,targetPosition=[0,0,0,1], targetVelocity=[0,0,0], positionGain=0,velocityGain=1,force=[jointFrictionForce,jointFrictionForce,0])
		
    self._jointDofCounts=[4,4,4,1,4,4,1,4,1,4,4,1]
    
    #only those body parts/links are allowed to touch the ground, otherwise the episode terminates
    self._allowed_body_parts=[5,11]
    
    #[x,y,z] base position and [x,y,z,w] base orientation!
    self._totalDofs = 7 
    for dof in self._jointDofCounts:
      self._totalDofs += dof
    self.setSimTime(0)
    
    self.resetPose()
    
  def resetPose(self):
    print("resetPose with self._frame=", self._frame, " and self._frameFraction=",self._frameFraction)
    pose = self.computePose(self._frameFraction)
    self.initializePose(self._poseInterpolator, self._sim_model, initBase=True)
    self.initializePose(self._poseInterpolator, self._kin_model, initBase=False)
    
  def initializePose(self,  pose, phys_model,initBase, initializeVelocity = True):
    
    if initializeVelocity:
      if initBase:
        self._pybullet_client.resetBasePositionAndOrientation(phys_model, pose._basePos, pose._baseOrn)
        self._pybullet_client.resetBaseVelocity(phys_model, pose._baseLinVel, pose._baseAngVel)
      self._pybullet_client.resetJointStateMultiDof(phys_model,chest,pose._chestRot, pose._chestVel)
      self._pybullet_client.resetJointStateMultiDof(phys_model,neck,pose._neckRot, pose._neckVel)
      self._pybullet_client.resetJointStateMultiDof(phys_model,rightHip,pose._rightHipRot, pose._rightHipVel)
      self._pybullet_client.resetJointStateMultiDof(phys_model,rightKnee,pose._rightKneeRot, pose._rightKneeVel)
      self._pybullet_client.resetJointStateMultiDof(phys_model,rightAnkle,pose._rightAnkleRot, pose._rightAnkleVel)
      self._pybullet_client.resetJointStateMultiDof(phys_model,rightShoulder,pose._rightShoulderRot, pose._rightShoulderVel)
      self._pybullet_client.resetJointStateMultiDof(phys_model,rightElbow, pose._rightElbowRot, pose._rightElbowVel)
      self._pybullet_client.resetJointStateMultiDof(phys_model,leftHip, pose._leftHipRot, pose._leftHipVel)
      self._pybullet_client.resetJointStateMultiDof(phys_model,leftKnee, pose._leftKneeRot, pose._leftKneeVel)
      self._pybullet_client.resetJointStateMultiDof(phys_model,leftAnkle, pose._leftAnkleRot, pose._leftAnkleVel)
      self._pybullet_client.resetJointStateMultiDof(phys_model,leftShoulder, pose._leftShoulderRot, pose._leftShoulderVel)
      self._pybullet_client.resetJointStateMultiDof(phys_model,leftElbow, pose._leftElbowRot, pose._leftElbowVel)
    else:
      if initBase:
        self._pybullet_client.resetBasePositionAndOrientation(phys_model, pose._basePos, pose._baseOrn)
      self._pybullet_client.resetJointStateMultiDof(phys_model,chest,pose._chestRot, [0,0,0])
      self._pybullet_client.resetJointStateMultiDof(phys_model,neck,pose._neckRot, [0,0,0])
      self._pybullet_client.resetJointStateMultiDof(phys_model,rightHip,pose._rightHipRot, [0,0,0])
      self._pybullet_client.resetJointStateMultiDof(phys_model,rightKnee,pose._rightKneeRot, [0])
      self._pybullet_client.resetJointStateMultiDof(phys_model,rightAnkle,pose._rightAnkleRot, [0,0,0])
      self._pybullet_client.resetJointStateMultiDof(phys_model,rightShoulder,pose._rightShoulderRot, [0,0,0])
      self._pybullet_client.resetJointStateMultiDof(phys_model,rightElbow, pose._rightElbowRot, [0])
      self._pybullet_client.resetJointStateMultiDof(phys_model,leftHip, pose._leftHipRot, [0,0,0])
      self._pybullet_client.resetJointStateMultiDof(phys_model,leftKnee, pose._leftKneeRot, [0])
      self._pybullet_client.resetJointStateMultiDof(phys_model,leftAnkle, pose._leftAnkleRot, [0,0,0])
      self._pybullet_client.resetJointStateMultiDof(phys_model,leftShoulder, pose._leftShoulderRot, [0,0,0])
      self._pybullet_client.resetJointStateMultiDof(phys_model,leftElbow, pose._leftElbowRot, [0])    

  def calcCycleCount(self, simTime, cycleTime):
    phases = simTime / cycleTime;
    count = math.floor(phases)
    loop = True
    #count = (loop) ? count : cMathUtil::Clamp(count, 0, 1);
    return count
    

  def getCycleTime(self):
    keyFrameDuration = self._mocap_data.KeyFrameDuraction()
    cycleTime = keyFrameDuration*(self._mocap_data.NumFrames()-1)
    return cycleTime
    
  def setSimTime(self, t):
    self._simTime = t
    #print("SetTimeTime time =",t)
    keyFrameDuration = self._mocap_data.KeyFrameDuraction()
    cycleTime = self.getCycleTime()
    #print("self._motion_data.NumFrames()=",self._mocap_data.NumFrames())
    cycles = self.calcCycleCount(t, cycleTime)
    #print("cycles=",cycles)
    frameTime = t - cycles*cycleTime
    if (frameTime<0):
      frameTime += cycleTime
    
    #print("keyFrameDuration=",keyFrameDuration)  
    #print("frameTime=",frameTime)
    self._frame = int(frameTime/keyFrameDuration)
    #print("self._frame=",self._frame)
    
    self._frameNext = self._frame+1
    if (self._frameNext >=  self._mocap_data.NumFrames()):
      self._frameNext = self._frame

    self._frameFraction = (frameTime - self._frame*keyFrameDuration)/(keyFrameDuration)
    
  def computePose(self, frameFraction):
    frameData = self._mocap_data._motion_data['Frames'][self._frame]
    frameDataNext = self._mocap_data._motion_data['Frames'][self._frameNext]
    
    pose = self._poseInterpolator.Slerp(frameFraction, frameData, frameDataNext, self._pybullet_client)
    #print("self._poseInterpolator.Slerp(", frameFraction,")=", pose)
    return pose


  def convertActionToPose(self, action):
    pose = self._poseInterpolator.ConvertFromAction(self._pybullet_client, action)
    return pose      

  def computePDForces(self, desiredPositions, desiredVelocities, maxForces):
    if desiredVelocities==None:
      desiredVelocities = [0]*self._totalDofs
      
      
    taus = self._stablePD.computePD(bodyUniqueId=self._sim_model, 
      jointIndices = self._jointIndicesAll,
      desiredPositions = desiredPositions,
      desiredVelocities = desiredVelocities,
      kps = self._kpOrg,
      kds = self._kdOrg,
      maxForces = maxForces, 
      timeStep=self._timeStep)
    return taus

  def applyPDForces(self, taus):
    dofIndex=7
    scaling = 1
    for index in range (len(self._jointIndicesAll)):
      jointIndex = self._jointIndicesAll[index]
      if self._jointDofCounts[index]==4:
         force=[scaling*taus[dofIndex+0],scaling*taus[dofIndex+1],scaling*taus[dofIndex+2]]
         #print("force[", jointIndex,"]=",force)
         self._pybullet_client.setJointMotorControlMultiDof(self._sim_model,jointIndex,self._pybullet_client.TORQUE_CONTROL,force=force)
      if self._jointDofCounts[index]==1:
         force=[scaling*taus[dofIndex]]
         #print("force[", jointIndex,"]=",force)
         self._pybullet_client.setJointMotorControlMultiDof(self._sim_model, jointIndex, controlMode=self._pybullet_client.TORQUE_CONTROL, force=force)
      dofIndex+=self._jointDofCounts[index]
      
      
      
  def getPhase(self):
    keyFrameDuration = self._mocap_data.KeyFrameDuraction()
    cycleTime = keyFrameDuration*(self._mocap_data.NumFrames()-1)
    phase = self._simTime / cycleTime
    phase = math.fmod(phase,1.0)
    if (phase<0):
      phase += 1
    return phase

  def buildHeadingTrans(self, rootOrn):
    #align root transform 'forward' with world-space x axis
    eul = self._pybullet_client.getEulerFromQuaternion(rootOrn)
    refDir = [1,0,0]
    rotVec = self._pybullet_client.rotateVector(rootOrn, refDir)
    heading = math.atan2(-rotVec[2], rotVec[0])
    heading2=eul[1]
    #print("heading=",heading)
    headingOrn = self._pybullet_client.getQuaternionFromAxisAngle([0,1,0],-heading)
    return headingOrn


  def buildOriginTrans(self):
    rootPos,rootOrn = self._pybullet_client.getBasePositionAndOrientation(self._sim_model)
    
    #print("rootPos=",rootPos, " rootOrn=",rootOrn)
    invRootPos=[-rootPos[0], 0, -rootPos[2]]
    #invOrigTransPos, invOrigTransOrn = self._pybullet_client.invertTransform(rootPos,rootOrn)
    headingOrn = self.buildHeadingTrans(rootOrn)
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

  def getState(self):

    stateVector = []
    phase = self.getPhase()
    #print("phase=",phase)
    stateVector.append(phase)
    
    rootTransPos, rootTransOrn=self.buildOriginTrans()
    basePos,baseOrn = self._pybullet_client.getBasePositionAndOrientation(self._sim_model)
    
    rootPosRel, dummy = self._pybullet_client.multiplyTransforms(rootTransPos, rootTransOrn, basePos,[0,0,0,1])
    #print("!!!rootPosRel =",rootPosRel )
    #print("rootTransPos=",rootTransPos)
    #print("basePos=",basePos)
    localPos,localOrn = self._pybullet_client.multiplyTransforms( rootTransPos, rootTransOrn , basePos,baseOrn )
    
    localPos=[localPos[0]-rootPosRel[0],localPos[1]-rootPosRel[1],localPos[2]-rootPosRel[2]]
    #print("localPos=",localPos)
        
    stateVector.append(rootPosRel[1])
    
    #self.pb2dmJoints=[0,1,2,9,10,11,3,4,5,12,13,14,6,7,8]
    self.pb2dmJoints=[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14]
    
    for pbJoint in range (self._pybullet_client.getNumJoints(self._sim_model)):
      j = self.pb2dmJoints[pbJoint]
      #print("joint order:",j)
      ls = self._pybullet_client.getLinkState(self._sim_model, j, computeForwardKinematics=True)
      linkPos = ls[0]
      linkOrn = ls[1]
      linkPosLocal, linkOrnLocal = self._pybullet_client.multiplyTransforms(rootTransPos, rootTransOrn, linkPos,linkOrn)
      if (linkOrnLocal[3]<0):
        linkOrnLocal=[-linkOrnLocal[0],-linkOrnLocal[1],-linkOrnLocal[2],-linkOrnLocal[3]]
      linkPosLocal=[linkPosLocal[0]-rootPosRel[0],linkPosLocal[1]-rootPosRel[1],linkPosLocal[2]-rootPosRel[2]]
      for l in linkPosLocal:
        stateVector.append(l)
      #re-order the quaternion, DeepMimic uses w,x,y,z
      
      if (linkOrnLocal[3]<0):
        linkOrnLocal[0]*=-1
        linkOrnLocal[1]*=-1
        linkOrnLocal[2]*=-1
        linkOrnLocal[3]*=-1
        
      stateVector.append(linkOrnLocal[3])
      stateVector.append(linkOrnLocal[0])
      stateVector.append(linkOrnLocal[1])
      stateVector.append(linkOrnLocal[2])
    
    
    for pbJoint in range (self._pybullet_client.getNumJoints(self._sim_model)):
      j = self.pb2dmJoints[pbJoint]
      ls = self._pybullet_client.getLinkState(self._sim_model, j, computeLinkVelocity=True)
      linkLinVel = ls[6]
      linkAngVel = ls[7]
      linkLinVelLocal  , unused = self._pybullet_client.multiplyTransforms([0,0,0], rootTransOrn, linkLinVel,[0,0,0,1])
      #linkLinVelLocal=[linkLinVelLocal[0]-rootPosRel[0],linkLinVelLocal[1]-rootPosRel[1],linkLinVelLocal[2]-rootPosRel[2]]
      linkAngVelLocal ,unused = self._pybullet_client.multiplyTransforms([0,0,0], rootTransOrn, linkAngVel,[0,0,0,1])
      
      for l in linkLinVelLocal:
        stateVector.append(l)
      for l in linkAngVelLocal:
        stateVector.append(l)
    
    #print("stateVector len=",len(stateVector))  
    #for st in range (len(stateVector)):
    #  print("state[",st,"]=",stateVector[st])
    return stateVector
  
  def terminates(self):
    #check if any non-allowed body part hits the ground
    terminates=False
    pts = self._pybullet_client.getContactPoints()
    for p in pts:
      part = -1
      #ignore self-collision
      if (p[1]==p[2]):
        continue
      if (p[1]==self._sim_model):
        part=p[3]
      if (p[2]==self._sim_model):
        part=p[4]
      if (part >=0 and part not in self._allowed_body_parts):
        #print("terminating part:", part)
        terminates=True
      
    return terminates
    
  def getReward(self, pose):
    #from DeepMimic double cSceneImitate::CalcRewardImitate
    pose_w = 0.5
    vel_w = 0.05
    end_eff_w = 0 #0.15
    root_w = 0#0.2
    com_w = 0#0.1

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
    
    #pose = self.InitializePoseFromMotionData()
    #print("self._kin_model=",self._kin_model)
    #print("kinematicHumanoid #joints=",self._pybullet_client.getNumJoints(self._kin_model))
    #self.ApplyPose(pose, True, True, self._kin_model, self._pybullet_client)

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
      
      simJointInfo = self._pybullet_client.getJointStateMultiDof(self._sim_model, j)
      
      #print("simJointInfo.pos=",simJointInfo[0])
      #print("simJointInfo.vel=",simJointInfo[1])
      kinJointInfo = self._pybullet_client.getJointStateMultiDof(self._kin_model,j)
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
