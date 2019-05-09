from pybullet_utils import pd_controller_stable
from pybullet_envs.deep_mimic.env import quadruped_pose_interpolator
import math


class QuadrupedStablePD(object):

  def __init__(self, pybullet_client, mocap_data, timeStep, useFixedBase=True):
    self._pybullet_client = pybullet_client
    self._mocap_data = mocap_data
    print("LOADING quadruped!")

    startPos = [0.007058990464444105, 0.03149299192130908, 0.4918981912395484]
    startOrn = [
        0.005934649695708604, 0.7065453990917289, 0.7076373820553712, -0.0027774940359030264
    ]
    self._sim_model = self._pybullet_client.loadURDF("laikago/laikago.urdf",
                                                     startPos,
                                                     startOrn,
                                                     flags=urdfFlags,
                                                     useFixedBase=False)
    self._pybullet_client.resetBasePositionAndOrientation(_sim_model, startPos, startOrn)

    self._end_effectors = []  #ankle and wrist, both left and right

    self._kin_model = self._pybullet_client.loadURDF("laikago/laikago.urdf",
                                                     startPos,
                                                     startOrn,
                                                     useFixedBase=True)

    self._pybullet_client.changeDynamics(self._sim_model, -1, lateralFriction=0.9)
    for j in range(self._pybullet_client.getNumJoints(self._sim_model)):
      self._pybullet_client.changeDynamics(self._sim_model, j, lateralFriction=0.9)

    self._pybullet_client.changeDynamics(self._sim_model, -1, linearDamping=0, angularDamping=0)
    self._pybullet_client.changeDynamics(self._kin_model, -1, linearDamping=0, angularDamping=0)

    #todo: add feature to disable simulation for a particular object. Until then, disable all collisions
    self._pybullet_client.setCollisionFilterGroupMask(self._kin_model,
                                                      -1,
                                                      collisionFilterGroup=0,
                                                      collisionFilterMask=0)
    self._pybullet_client.changeDynamics(
        self._kin_model,
        -1,
        activationState=self._pybullet_client.ACTIVATION_STATE_SLEEP +
        self._pybullet_client.ACTIVATION_STATE_ENABLE_SLEEPING +
        self._pybullet_client.ACTIVATION_STATE_DISABLE_WAKEUP)
    alpha = 0.4
    self._pybullet_client.changeVisualShape(self._kin_model, -1, rgbaColor=[1, 1, 1, alpha])
    for j in range(self._pybullet_client.getNumJoints(self._kin_model)):
      self._pybullet_client.setCollisionFilterGroupMask(self._kin_model,
                                                        j,
                                                        collisionFilterGroup=0,
                                                        collisionFilterMask=0)
      self._pybullet_client.changeDynamics(
          self._kin_model,
          j,
          activationState=self._pybullet_client.ACTIVATION_STATE_SLEEP +
          self._pybullet_client.ACTIVATION_STATE_ENABLE_SLEEPING +
          self._pybullet_client.ACTIVATION_STATE_DISABLE_WAKEUP)
      self._pybullet_client.changeVisualShape(self._kin_model, j, rgbaColor=[1, 1, 1, alpha])

    self._poseInterpolator = humanoid_pose_interpolator.HumanoidPoseInterpolator()

    for i in range(self._mocap_data.NumFrames() - 1):
      frameData = self._mocap_data._motion_data['Frames'][i]
      self._poseInterpolator.PostProcessMotionData(frameData)

    self._stablePD = pd_controller_stable.PDControllerStableMultiDof(self._pybullet_client)
    self._timeStep = timeStep
    #todo: kp/pd
    self._kpOrg = [
        0, 0, 0, 0, 0, 0, 0, 1000, 1000, 1000, 1000, 100, 100, 100, 100, 500, 500, 500, 500, 500,
        400, 400, 400, 400, 400, 400, 400, 400, 300, 500, 500, 500, 500, 500, 400, 400, 400, 400,
        400, 400, 400, 400, 300
    ]
    self._kdOrg = [
        0, 0, 0, 0, 0, 0, 0, 100, 100, 100, 100, 10, 10, 10, 10, 50, 50, 50, 50, 50, 40, 40, 40,
        40, 40, 40, 40, 40, 30, 50, 50, 50, 50, 50, 40, 40, 40, 40, 40, 40, 40, 40, 30
    ]

    self._jointIndicesAll = [
        chest, neck, rightHip, rightKnee, rightAnkle, rightShoulder, rightElbow, leftHip, leftKnee,
        leftAnkle, leftShoulder, leftElbow
    ]
    for j in self._jointIndicesAll:
      #self._pybullet_client.setJointMotorControlMultiDof(self._sim_model, j, self._pybullet_client.POSITION_CONTROL, force=[1,1,1])
      self._pybullet_client.setJointMotorControl2(self._sim_model,
                                                  j,
                                                  self._pybullet_client.POSITION_CONTROL,
                                                  targetPosition=0,
                                                  positionGain=0,
                                                  targetVelocity=0,
                                                  force=jointFrictionForce)
      self._pybullet_client.setJointMotorControlMultiDof(
          self._sim_model,
          j,
          self._pybullet_client.POSITION_CONTROL,
          targetPosition=[0, 0, 0, 1],
          targetVelocity=[0, 0, 0],
          positionGain=0,
          velocityGain=1,
          force=[jointFrictionForce, jointFrictionForce, jointFrictionForce])
      self._pybullet_client.setJointMotorControl2(self._kin_model,
                                                  j,
                                                  self._pybullet_client.POSITION_CONTROL,
                                                  targetPosition=0,
                                                  positionGain=0,
                                                  targetVelocity=0,
                                                  force=0)
      self._pybullet_client.setJointMotorControlMultiDof(
          self._kin_model,
          j,
          self._pybullet_client.POSITION_CONTROL,
          targetPosition=[0, 0, 0, 1],
          targetVelocity=[0, 0, 0],
          positionGain=0,
          velocityGain=1,
          force=[jointFrictionForce, jointFrictionForce, 0])

    self._jointDofCounts = [4, 4, 4, 1, 4, 4, 1, 4, 1, 4, 4, 1]

    #only those body parts/links are allowed to touch the ground, otherwise the episode terminates
    self._allowed_body_parts = [5, 11]

    #[x,y,z] base position and [x,y,z,w] base orientation!
    self._totalDofs = 7
    for dof in self._jointDofCounts:
      self._totalDofs += dof
    self.setSimTime(0)

    self.resetPose()

  def resetPose(self):
    #print("resetPose with self._frame=", self._frame, " and self._frameFraction=",self._frameFraction)
    pose = self.computePose(self._frameFraction)
    self.initializePose(self._poseInterpolator, self._sim_model, initBase=True)
    self.initializePose(self._poseInterpolator, self._kin_model, initBase=False)

  def initializePose(self, pose, phys_model, initBase, initializeVelocity=True):

    if initializeVelocity:
      if initBase:
        self._pybullet_client.resetBasePositionAndOrientation(phys_model, pose._basePos,
                                                              pose._baseOrn)
        self._pybullet_client.resetBaseVelocity(phys_model, pose._baseLinVel, pose._baseAngVel)
      self._pybullet_client.resetJointStateMultiDof(phys_model, chest, pose._chestRot,
                                                    pose._chestVel)
      self._pybullet_client.resetJointStateMultiDof(phys_model, neck, pose._neckRot, pose._neckVel)
      self._pybullet_client.resetJointStateMultiDof(phys_model, rightHip, pose._rightHipRot,
                                                    pose._rightHipVel)
      self._pybullet_client.resetJointStateMultiDof(phys_model, rightKnee, pose._rightKneeRot,
                                                    pose._rightKneeVel)
      self._pybullet_client.resetJointStateMultiDof(phys_model, rightAnkle, pose._rightAnkleRot,
                                                    pose._rightAnkleVel)
      self._pybullet_client.resetJointStateMultiDof(phys_model, rightShoulder,
                                                    pose._rightShoulderRot, pose._rightShoulderVel)
      self._pybullet_client.resetJointStateMultiDof(phys_model, rightElbow, pose._rightElbowRot,
                                                    pose._rightElbowVel)
      self._pybullet_client.resetJointStateMultiDof(phys_model, leftHip, pose._leftHipRot,
                                                    pose._leftHipVel)
      self._pybullet_client.resetJointStateMultiDof(phys_model, leftKnee, pose._leftKneeRot,
                                                    pose._leftKneeVel)
      self._pybullet_client.resetJointStateMultiDof(phys_model, leftAnkle, pose._leftAnkleRot,
                                                    pose._leftAnkleVel)
      self._pybullet_client.resetJointStateMultiDof(phys_model, leftShoulder,
                                                    pose._leftShoulderRot, pose._leftShoulderVel)
      self._pybullet_client.resetJointStateMultiDof(phys_model, leftElbow, pose._leftElbowRot,
                                                    pose._leftElbowVel)
    else:
      if initBase:
        self._pybullet_client.resetBasePositionAndOrientation(phys_model, pose._basePos,
                                                              pose._baseOrn)
      self._pybullet_client.resetJointStateMultiDof(phys_model, chest, pose._chestRot, [0, 0, 0])
      self._pybullet_client.resetJointStateMultiDof(phys_model, neck, pose._neckRot, [0, 0, 0])
      self._pybullet_client.resetJointStateMultiDof(phys_model, rightHip, pose._rightHipRot,
                                                    [0, 0, 0])
      self._pybullet_client.resetJointStateMultiDof(phys_model, rightKnee, pose._rightKneeRot, [0])
      self._pybullet_client.resetJointStateMultiDof(phys_model, rightAnkle, pose._rightAnkleRot,
                                                    [0, 0, 0])
      self._pybullet_client.resetJointStateMultiDof(phys_model, rightShoulder,
                                                    pose._rightShoulderRot, [0, 0, 0])
      self._pybullet_client.resetJointStateMultiDof(phys_model, rightElbow, pose._rightElbowRot,
                                                    [0])
      self._pybullet_client.resetJointStateMultiDof(phys_model, leftHip, pose._leftHipRot,
                                                    [0, 0, 0])
      self._pybullet_client.resetJointStateMultiDof(phys_model, leftKnee, pose._leftKneeRot, [0])
      self._pybullet_client.resetJointStateMultiDof(phys_model, leftAnkle, pose._leftAnkleRot,
                                                    [0, 0, 0])
      self._pybullet_client.resetJointStateMultiDof(phys_model, leftShoulder,
                                                    pose._leftShoulderRot, [0, 0, 0])
      self._pybullet_client.resetJointStateMultiDof(phys_model, leftElbow, pose._leftElbowRot, [0])

  def calcCycleCount(self, simTime, cycleTime):
    phases = simTime / cycleTime
    count = math.floor(phases)
    loop = True
    #count = (loop) ? count : cMathUtil::Clamp(count, 0, 1);
    return count

  def getCycleTime(self):
    keyFrameDuration = self._mocap_data.KeyFrameDuraction()
    cycleTime = keyFrameDuration * (self._mocap_data.NumFrames() - 1)
    return cycleTime

  def setSimTime(self, t):
    self._simTime = t
    #print("SetTimeTime time =",t)
    keyFrameDuration = self._mocap_data.KeyFrameDuraction()
    cycleTime = self.getCycleTime()
    #print("self._motion_data.NumFrames()=",self._mocap_data.NumFrames())
    self._cycleCount = self.calcCycleCount(t, cycleTime)
    #print("cycles=",cycles)
    frameTime = t - self._cycleCount * cycleTime
    if (frameTime < 0):
      frameTime += cycleTime

    #print("keyFrameDuration=",keyFrameDuration)
    #print("frameTime=",frameTime)
    self._frame = int(frameTime / keyFrameDuration)
    #print("self._frame=",self._frame)

    self._frameNext = self._frame + 1
    if (self._frameNext >= self._mocap_data.NumFrames()):
      self._frameNext = self._frame

    self._frameFraction = (frameTime - self._frame * keyFrameDuration) / (keyFrameDuration)

  def computeCycleOffset(self):
    firstFrame = 0
    lastFrame = self._mocap_data.NumFrames() - 1
    frameData = self._mocap_data._motion_data['Frames'][0]
    frameDataNext = self._mocap_data._motion_data['Frames'][lastFrame]

    basePosStart = [frameData[1], frameData[2], frameData[3]]
    basePosEnd = [frameDataNext[1], frameDataNext[2], frameDataNext[3]]
    self._cycleOffset = [
        basePosEnd[0] - basePosStart[0], basePosEnd[1] - basePosStart[1],
        basePosEnd[2] - basePosStart[2]
    ]
    return self._cycleOffset

  def computePose(self, frameFraction):
    frameData = self._mocap_data._motion_data['Frames'][self._frame]
    frameDataNext = self._mocap_data._motion_data['Frames'][self._frameNext]

    self._poseInterpolator.Slerp(frameFraction, frameData, frameDataNext, self._pybullet_client)
    #print("self._poseInterpolator.Slerp(", frameFraction,")=", pose)
    self.computeCycleOffset()
    oldPos = self._poseInterpolator._basePos
    self._poseInterpolator._basePos = [
        oldPos[0] + self._cycleCount * self._cycleOffset[0],
        oldPos[1] + self._cycleCount * self._cycleOffset[1],
        oldPos[2] + self._cycleCount * self._cycleOffset[2]
    ]
    pose = self._poseInterpolator.GetPose()

    return pose

  def convertActionToPose(self, action):
    pose = self._poseInterpolator.ConvertFromAction(self._pybullet_client, action)
    return pose

  def computePDForces(self, desiredPositions, desiredVelocities, maxForces):
    if desiredVelocities == None:
      desiredVelocities = [0] * self._totalDofs

    taus = self._stablePD.computePD(bodyUniqueId=self._sim_model,
                                    jointIndices=self._jointIndicesAll,
                                    desiredPositions=desiredPositions,
                                    desiredVelocities=desiredVelocities,
                                    kps=self._kpOrg,
                                    kds=self._kdOrg,
                                    maxForces=maxForces,
                                    timeStep=self._timeStep)
    return taus

  def applyPDForces(self, taus):
    dofIndex = 7
    scaling = 1
    for index in range(len(self._jointIndicesAll)):
      jointIndex = self._jointIndicesAll[index]
      if self._jointDofCounts[index] == 4:
        force = [
            scaling * taus[dofIndex + 0], scaling * taus[dofIndex + 1],
            scaling * taus[dofIndex + 2]
        ]
        #print("force[", jointIndex,"]=",force)
        self._pybullet_client.setJointMotorControlMultiDof(self._sim_model,
                                                           jointIndex,
                                                           self._pybullet_client.TORQUE_CONTROL,
                                                           force=force)
      if self._jointDofCounts[index] == 1:
        force = [scaling * taus[dofIndex]]
        #print("force[", jointIndex,"]=",force)
        self._pybullet_client.setJointMotorControlMultiDof(
            self._sim_model,
            jointIndex,
            controlMode=self._pybullet_client.TORQUE_CONTROL,
            force=force)
      dofIndex += self._jointDofCounts[index]

  def setJointMotors(self, desiredPositions, maxForces):
    controlMode = self._pybullet_client.POSITION_CONTROL
    startIndex = 7
    chest = 1
    neck = 2
    rightHip = 3
    rightKnee = 4
    rightAnkle = 5
    rightShoulder = 6
    rightElbow = 7
    leftHip = 9
    leftKnee = 10
    leftAnkle = 11
    leftShoulder = 12
    leftElbow = 13
    kp = 0.2

    forceScale = 1
    #self._jointDofCounts=[4,4,4,1,4,4,1,4,1,4,4,1]
    maxForce = [
        forceScale * maxForces[startIndex], forceScale * maxForces[startIndex + 1],
        forceScale * maxForces[startIndex + 2], forceScale * maxForces[startIndex + 3]
    ]
    startIndex += 4
    self._pybullet_client.setJointMotorControlMultiDof(
        self._sim_model,
        chest,
        controlMode,
        targetPosition=self._poseInterpolator._chestRot,
        positionGain=kp,
        force=maxForce)
    maxForce = [
        maxForces[startIndex], maxForces[startIndex + 1], maxForces[startIndex + 2],
        maxForces[startIndex + 3]
    ]
    startIndex += 4
    self._pybullet_client.setJointMotorControlMultiDof(
        self._sim_model,
        neck,
        controlMode,
        targetPosition=self._poseInterpolator._neckRot,
        positionGain=kp,
        force=maxForce)
    maxForce = [
        maxForces[startIndex], maxForces[startIndex + 1], maxForces[startIndex + 2],
        maxForces[startIndex + 3]
    ]
    startIndex += 4
    self._pybullet_client.setJointMotorControlMultiDof(
        self._sim_model,
        rightHip,
        controlMode,
        targetPosition=self._poseInterpolator._rightHipRot,
        positionGain=kp,
        force=maxForce)
    maxForce = [forceScale * maxForces[startIndex]]
    startIndex += 1
    self._pybullet_client.setJointMotorControlMultiDof(
        self._sim_model,
        rightKnee,
        controlMode,
        targetPosition=self._poseInterpolator._rightKneeRot,
        positionGain=kp,
        force=maxForce)
    maxForce = [
        maxForces[startIndex], maxForces[startIndex + 1], maxForces[startIndex + 2],
        maxForces[startIndex + 3]
    ]
    startIndex += 4
    self._pybullet_client.setJointMotorControlMultiDof(
        self._sim_model,
        rightAnkle,
        controlMode,
        targetPosition=self._poseInterpolator._rightAnkleRot,
        positionGain=kp,
        force=maxForce)
    maxForce = [
        forceScale * maxForces[startIndex], forceScale * maxForces[startIndex + 1],
        forceScale * maxForces[startIndex + 2], forceScale * maxForces[startIndex + 3]
    ]
    startIndex += 4
    maxForce = [forceScale * maxForces[startIndex]]
    startIndex += 1
    self._pybullet_client.setJointMotorControlMultiDof(
        self._sim_model,
        rightElbow,
        controlMode,
        targetPosition=self._poseInterpolator._rightElbowRot,
        positionGain=kp,
        force=maxForce)
    maxForce = [
        maxForces[startIndex], maxForces[startIndex + 1], maxForces[startIndex + 2],
        maxForces[startIndex + 3]
    ]
    startIndex += 4
    self._pybullet_client.setJointMotorControlMultiDof(
        self._sim_model,
        leftHip,
        controlMode,
        targetPosition=self._poseInterpolator._leftHipRot,
        positionGain=kp,
        force=maxForce)
    maxForce = [forceScale * maxForces[startIndex]]
    startIndex += 1
    self._pybullet_client.setJointMotorControlMultiDof(
        self._sim_model,
        leftKnee,
        controlMode,
        targetPosition=self._poseInterpolator._leftKneeRot,
        positionGain=kp,
        force=maxForce)
    maxForce = [
        maxForces[startIndex], maxForces[startIndex + 1], maxForces[startIndex + 2],
        maxForces[startIndex + 3]
    ]
    startIndex += 4
    self._pybullet_client.setJointMotorControlMultiDof(
        self._sim_model,
        leftAnkle,
        controlMode,
        targetPosition=self._poseInterpolator._leftAnkleRot,
        positionGain=kp,
        force=maxForce)
    maxForce = [
        maxForces[startIndex], maxForces[startIndex + 1], maxForces[startIndex + 2],
        maxForces[startIndex + 3]
    ]
    startIndex += 4
    self._pybullet_client.setJointMotorControlMultiDof(
        self._sim_model,
        leftShoulder,
        controlMode,
        targetPosition=self._poseInterpolator._leftShoulderRot,
        positionGain=kp,
        force=maxForce)
    maxForce = [forceScale * maxForces[startIndex]]
    startIndex += 1
    self._pybullet_client.setJointMotorControlMultiDof(
        self._sim_model,
        leftElbow,
        controlMode,
        targetPosition=self._poseInterpolator._leftElbowRot,
        positionGain=kp,
        force=maxForce)
    #print("startIndex=",startIndex)

  def getPhase(self):
    keyFrameDuration = self._mocap_data.KeyFrameDuraction()
    cycleTime = keyFrameDuration * (self._mocap_data.NumFrames() - 1)
    phase = self._simTime / cycleTime
    phase = math.fmod(phase, 1.0)
    if (phase < 0):
      phase += 1
    return phase

  def buildHeadingTrans(self, rootOrn):
    #align root transform 'forward' with world-space x axis
    eul = self._pybullet_client.getEulerFromQuaternion(rootOrn)
    refDir = [1, 0, 0]
    rotVec = self._pybullet_client.rotateVector(rootOrn, refDir)
    heading = math.atan2(-rotVec[2], rotVec[0])
    heading2 = eul[1]
    #print("heading=",heading)
    headingOrn = self._pybullet_client.getQuaternionFromAxisAngle([0, 1, 0], -heading)
    return headingOrn

  def buildOriginTrans(self):
    rootPos, rootOrn = self._pybullet_client.getBasePositionAndOrientation(self._sim_model)

    #print("rootPos=",rootPos, " rootOrn=",rootOrn)
    invRootPos = [-rootPos[0], 0, -rootPos[2]]
    #invOrigTransPos, invOrigTransOrn = self._pybullet_client.invertTransform(rootPos,rootOrn)
    headingOrn = self.buildHeadingTrans(rootOrn)
    #print("headingOrn=",headingOrn)
    headingMat = self._pybullet_client.getMatrixFromQuaternion(headingOrn)
    #print("headingMat=",headingMat)
    #dummy, rootOrnWithoutHeading = self._pybullet_client.multiplyTransforms([0,0,0],headingOrn, [0,0,0], rootOrn)
    #dummy, invOrigTransOrn = self._pybullet_client.multiplyTransforms([0,0,0],rootOrnWithoutHeading, invOrigTransPos, invOrigTransOrn)

    invOrigTransPos, invOrigTransOrn = self._pybullet_client.multiplyTransforms([0, 0, 0],
                                                                                headingOrn,
                                                                                invRootPos,
                                                                                [0, 0, 0, 1])
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

    rootTransPos, rootTransOrn = self.buildOriginTrans()
    basePos, baseOrn = self._pybullet_client.getBasePositionAndOrientation(self._sim_model)

    rootPosRel, dummy = self._pybullet_client.multiplyTransforms(rootTransPos, rootTransOrn,
                                                                 basePos, [0, 0, 0, 1])
    #print("!!!rootPosRel =",rootPosRel )
    #print("rootTransPos=",rootTransPos)
    #print("basePos=",basePos)
    localPos, localOrn = self._pybullet_client.multiplyTransforms(rootTransPos, rootTransOrn,
                                                                  basePos, baseOrn)

    localPos = [
        localPos[0] - rootPosRel[0], localPos[1] - rootPosRel[1], localPos[2] - rootPosRel[2]
    ]
    #print("localPos=",localPos)

    stateVector.append(rootPosRel[1])

    #self.pb2dmJoints=[0,1,2,9,10,11,3,4,5,12,13,14,6,7,8]
    self.pb2dmJoints = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]

    for pbJoint in range(self._pybullet_client.getNumJoints(self._sim_model)):
      j = self.pb2dmJoints[pbJoint]
      #print("joint order:",j)
      ls = self._pybullet_client.getLinkState(self._sim_model, j, computeForwardKinematics=True)
      linkPos = ls[0]
      linkOrn = ls[1]
      linkPosLocal, linkOrnLocal = self._pybullet_client.multiplyTransforms(
          rootTransPos, rootTransOrn, linkPos, linkOrn)
      if (linkOrnLocal[3] < 0):
        linkOrnLocal = [-linkOrnLocal[0], -linkOrnLocal[1], -linkOrnLocal[2], -linkOrnLocal[3]]
      linkPosLocal = [
          linkPosLocal[0] - rootPosRel[0], linkPosLocal[1] - rootPosRel[1],
          linkPosLocal[2] - rootPosRel[2]
      ]
      for l in linkPosLocal:
        stateVector.append(l)
      #re-order the quaternion, DeepMimic uses w,x,y,z

      if (linkOrnLocal[3] < 0):
        linkOrnLocal[0] *= -1
        linkOrnLocal[1] *= -1
        linkOrnLocal[2] *= -1
        linkOrnLocal[3] *= -1

      stateVector.append(linkOrnLocal[3])
      stateVector.append(linkOrnLocal[0])
      stateVector.append(linkOrnLocal[1])
      stateVector.append(linkOrnLocal[2])

    for pbJoint in range(self._pybullet_client.getNumJoints(self._sim_model)):
      j = self.pb2dmJoints[pbJoint]
      ls = self._pybullet_client.getLinkState(self._sim_model, j, computeLinkVelocity=True)
      linkLinVel = ls[6]
      linkAngVel = ls[7]
      linkLinVelLocal, unused = self._pybullet_client.multiplyTransforms([0, 0, 0], rootTransOrn,
                                                                         linkLinVel, [0, 0, 0, 1])
      #linkLinVelLocal=[linkLinVelLocal[0]-rootPosRel[0],linkLinVelLocal[1]-rootPosRel[1],linkLinVelLocal[2]-rootPosRel[2]]
      linkAngVelLocal, unused = self._pybullet_client.multiplyTransforms([0, 0, 0], rootTransOrn,
                                                                         linkAngVel, [0, 0, 0, 1])

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
    terminates = False
    pts = self._pybullet_client.getContactPoints()
    for p in pts:
      part = -1
      #ignore self-collision
      if (p[1] == p[2]):
        continue
      if (p[1] == self._sim_model):
        part = p[3]
      if (p[2] == self._sim_model):
        part = p[4]
      if (part >= 0 and part not in self._allowed_body_parts):
        #print("terminating part:", part)
        terminates = True

    return terminates

  def quatMul(self, q1, q2):
    return [
        q1[3] * q2[0] + q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1],
        q1[3] * q2[1] + q1[1] * q2[3] + q1[2] * q2[0] - q1[0] * q2[2],
        q1[3] * q2[2] + q1[2] * q2[3] + q1[0] * q2[1] - q1[1] * q2[0],
        q1[3] * q2[3] - q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2]
    ]

  def calcRootAngVelErr(self, vel0, vel1):
    diff = [vel0[0] - vel1[0], vel0[1] - vel1[1], vel0[2] - vel1[2]]
    return diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]

  def calcRootRotDiff(self, orn0, orn1):
    orn0Conj = [-orn0[0], -orn0[1], -orn0[2], orn0[3]]
    q_diff = self.quatMul(orn1, orn0Conj)
    axis, angle = self._pybullet_client.getAxisAngleFromQuaternion(q_diff)
    return angle * angle

  def getReward(self, pose):
    #from DeepMimic double cSceneImitate::CalcRewardImitate
    #todo: compensate for ground height in some parts, once we move to non-flat terrain
    pose_w = 0.5
    vel_w = 0.05
    end_eff_w = 0.15
    root_w = 0.2
    com_w = 0  #0.1

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

    mJointWeights = [
        0.20833, 0.10416, 0.0625, 0.10416, 0.0625, 0.041666666666666671, 0.0625, 0.0416, 0.00,
        0.10416, 0.0625, 0.0416, 0.0625, 0.0416, 0.0000
    ]

    num_end_effs = 0
    num_joints = 15

    root_rot_w = mJointWeights[root_id]
    rootPosSim, rootOrnSim = self._pybullet_client.getBasePositionAndOrientation(self._sim_model)
    rootPosKin, rootOrnKin = self._pybullet_client.getBasePositionAndOrientation(self._kin_model)
    linVelSim, angVelSim = self._pybullet_client.getBaseVelocity(self._sim_model)
    linVelKin, angVelKin = self._pybullet_client.getBaseVelocity(self._kin_model)

    root_rot_err = self.calcRootRotDiff(rootOrnSim, rootOrnKin)
    pose_err += root_rot_w * root_rot_err

    root_vel_diff = [
        linVelSim[0] - linVelKin[0], linVelSim[1] - linVelKin[1], linVelSim[2] - linVelKin[2]
    ]
    root_vel_err = root_vel_diff[0] * root_vel_diff[0] + root_vel_diff[1] * root_vel_diff[
        1] + root_vel_diff[2] * root_vel_diff[2]

    root_ang_vel_err = self.calcRootAngVelErr(angVelSim, angVelKin)
    vel_err += root_rot_w * root_ang_vel_err

    for j in range(num_joints):
      curr_pose_err = 0
      curr_vel_err = 0
      w = mJointWeights[j]

      simJointInfo = self._pybullet_client.getJointStateMultiDof(self._sim_model, j)

      #print("simJointInfo.pos=",simJointInfo[0])
      #print("simJointInfo.vel=",simJointInfo[1])
      kinJointInfo = self._pybullet_client.getJointStateMultiDof(self._kin_model, j)
      #print("kinJointInfo.pos=",kinJointInfo[0])
      #print("kinJointInfo.vel=",kinJointInfo[1])
      if (len(simJointInfo[0]) == 1):
        angle = simJointInfo[0][0] - kinJointInfo[0][0]
        curr_pose_err = angle * angle
        velDiff = simJointInfo[1][0] - kinJointInfo[1][0]
        curr_vel_err = velDiff * velDiff
      if (len(simJointInfo[0]) == 4):
        #print("quaternion diff")
        diffQuat = self._pybullet_client.getDifferenceQuaternion(simJointInfo[0], kinJointInfo[0])
        axis, angle = self._pybullet_client.getAxisAngleFromQuaternion(diffQuat)
        curr_pose_err = angle * angle
        diffVel = [
            simJointInfo[1][0] - kinJointInfo[1][0], simJointInfo[1][1] - kinJointInfo[1][1],
            simJointInfo[1][2] - kinJointInfo[1][2]
        ]
        curr_vel_err = diffVel[0] * diffVel[0] + diffVel[1] * diffVel[1] + diffVel[2] * diffVel[2]

      pose_err += w * curr_pose_err
      vel_err += w * curr_vel_err

      is_end_eff = j in self._end_effectors
      if is_end_eff:

        linkStateSim = self._pybullet_client.getLinkState(self._sim_model, j)
        linkStateKin = self._pybullet_client.getLinkState(self._kin_model, j)
        linkPosSim = linkStateSim[0]
        linkPosKin = linkStateKin[0]
        linkPosDiff = [
            linkPosSim[0] - linkPosKin[0], linkPosSim[1] - linkPosKin[1],
            linkPosSim[2] - linkPosKin[2]
        ]
        curr_end_err = linkPosDiff[0] * linkPosDiff[0] + linkPosDiff[1] * linkPosDiff[
            1] + linkPosDiff[2] * linkPosDiff[2]
        end_eff_err += curr_end_err
        num_end_effs += 1

    if (num_end_effs > 0):
      end_eff_err /= num_end_effs

    #double root_ground_h0 = mGround->SampleHeight(sim_char.GetRootPos())
    #double root_ground_h1 = kin_char.GetOriginPos()[1]
    #root_pos0[1] -= root_ground_h0
    #root_pos1[1] -= root_ground_h1
    root_pos_diff = [
        rootPosSim[0] - rootPosKin[0], rootPosSim[1] - rootPosKin[1], rootPosSim[2] - rootPosKin[2]
    ]
    root_pos_err = root_pos_diff[0] * root_pos_diff[0] + root_pos_diff[1] * root_pos_diff[
        1] + root_pos_diff[2] * root_pos_diff[2]
    #
    #root_rot_err = cMathUtil::QuatDiffTheta(root_rot0, root_rot1)
    #root_rot_err *= root_rot_err

    #root_vel_err = (root_vel1 - root_vel0).squaredNorm()
    #root_ang_vel_err = (root_ang_vel1 - root_ang_vel0).squaredNorm()

    root_err = root_pos_err + 0.1 * root_rot_err + 0.01 * root_vel_err + 0.001 * root_ang_vel_err

    #com_err = 0.1 * (com_vel1_world - com_vel0_world).squaredNorm()

    #print("pose_err=",pose_err)
    #print("vel_err=",vel_err)
    pose_reward = math.exp(-err_scale * pose_scale * pose_err)
    vel_reward = math.exp(-err_scale * vel_scale * vel_err)
    end_eff_reward = math.exp(-err_scale * end_eff_scale * end_eff_err)
    root_reward = math.exp(-err_scale * root_scale * root_err)
    com_reward = math.exp(-err_scale * com_scale * com_err)

    reward = pose_w * pose_reward + vel_w * vel_reward + end_eff_w * end_eff_reward + root_w * root_reward + com_w * com_reward

    # pose_reward,vel_reward,end_eff_reward, root_reward, com_reward);
    #print("reward=",reward)
    #print("pose_reward=",pose_reward)
    #print("vel_reward=",vel_reward)
    #print("end_eff_reward=",end_eff_reward)
    #print("root_reward=",root_reward)
    #print("com_reward=",com_reward)

    return reward
