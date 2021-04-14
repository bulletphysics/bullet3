import pybullet as p
import json
import time
import pybullet_data


useGUI = True
if useGUI:
  p.connect(p.GUI)
else:
  p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

useZUp = False
useYUp = not useZUp
showJointMotorTorques = False

if useYUp:
  p.configureDebugVisualizer(p.COV_ENABLE_Y_AXIS_UP, 1)

from pybullet_examples.pdControllerExplicit import PDControllerExplicitMultiDof
from pybullet_examples.pdControllerStable import PDControllerStableMultiDof

explicitPD = PDControllerExplicitMultiDof(p)
stablePD = PDControllerStableMultiDof(p)

p.resetDebugVisualizerCamera(cameraDistance=7.4,
                             cameraYaw=-94,
                             cameraPitch=-14,
                             cameraTargetPosition=[0.24, -0.02, -0.09])

import pybullet_data
p.setTimeOut(10000)
useMotionCapture = False
useMotionCaptureReset = False  #not	useMotionCapture
useExplicitPD = True

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setPhysicsEngineParameter(numSolverIterations=30)
#p.setPhysicsEngineParameter(solverResidualThreshold=1e-30)

#explicit	PD control requires	small	timestep
timeStep = 1. / 600.
#timeStep	=	1./240.

p.setPhysicsEngineParameter(fixedTimeStep=timeStep)

path = pybullet_data.getDataPath() + "/data/motions/humanoid3d_backflip.txt"
#path	=	pybullet_data.getDataPath()+"/data/motions/humanoid3d_cartwheel.txt"
#path	=	pybullet_data.getDataPath()+"/data/motions/humanoid3d_walk.txt"

#p.loadURDF("plane.urdf",[0,0,-1.03])
print("path	=	", path)
with open(path, 'r') as f:
  motion_dict = json.load(f)
#print("motion_dict	=	", motion_dict)
print("len motion=", len(motion_dict))
print(motion_dict['Loop'])
numFrames = len(motion_dict['Frames'])
print("#frames = ", numFrames)

frameId = p.addUserDebugParameter("frame", 0, numFrames - 1, 0)

erpId = p.addUserDebugParameter("erp", 0, 1, 0.2)

kpMotorId = p.addUserDebugParameter("kpMotor", 0, 1, .2)
forceMotorId = p.addUserDebugParameter("forceMotor", 0, 2000, 1000)

jointTypes = [
    "JOINT_REVOLUTE", "JOINT_PRISMATIC", "JOINT_SPHERICAL", "JOINT_PLANAR", "JOINT_FIXED"
]

startLocations = [[0, 0, 2], [0, 0, 0], [0, 0, -2], [0, 0, -4], [0, 0, 4]]

p.addUserDebugText("Stable PD",
                   [startLocations[0][0], startLocations[0][1] + 1, startLocations[0][2]],
                   [0, 0, 0])
p.addUserDebugText("Spherical	Drive",
                   [startLocations[1][0], startLocations[1][1] + 1, startLocations[1][2]],
                   [0, 0, 0])
p.addUserDebugText("Explicit PD",
                   [startLocations[2][0], startLocations[2][1] + 1, startLocations[2][2]],
                   [0, 0, 0])
p.addUserDebugText("Kinematic",
                   [startLocations[3][0], startLocations[3][1] + 1, startLocations[3][2]],
                   [0, 0, 0])
p.addUserDebugText("Stable PD (Py)",
                   [startLocations[4][0], startLocations[4][1] + 1, startLocations[4][2]],
                   [0, 0, 0])
flags=p.URDF_MAINTAIN_LINK_ORDER+p.URDF_USE_SELF_COLLISION
humanoid = p.loadURDF("humanoid/humanoid.urdf",
                      startLocations[0],
                      globalScaling=0.25,
                      useFixedBase=False,
                      flags=flags)
humanoid2 = p.loadURDF("humanoid/humanoid.urdf",
                       startLocations[1],
                       globalScaling=0.25,
                       useFixedBase=False,
                       flags=flags)
humanoid3 = p.loadURDF("humanoid/humanoid.urdf",
                       startLocations[2],
                       globalScaling=0.25,
                       useFixedBase=False,
                       flags=flags)
humanoid4 = p.loadURDF("humanoid/humanoid.urdf",
                       startLocations[3],
                       globalScaling=0.25,
                       useFixedBase=False,
                       flags=flags)
humanoid5 = p.loadURDF("humanoid/humanoid.urdf",
                      startLocations[4],
                      globalScaling=0.25,
                      useFixedBase=False,
                      flags=flags)

humanoid_fix = p.createConstraint(humanoid, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0],
                                  startLocations[0], [0, 0, 0, 1])
humanoid2_fix = p.createConstraint(humanoid2, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0],
                                   startLocations[1], [0, 0, 0, 1])
humanoid3_fix = p.createConstraint(humanoid3, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0],
                                   startLocations[2], [0, 0, 0, 1])
humanoid3_fix = p.createConstraint(humanoid4, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0],
                                   startLocations[3], [0, 0, 0, 1])
humanoid4_fix = p.createConstraint(humanoid5, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0],
                                   startLocations[4], [0, 0, 0, 1])

startPose = [
    2, 0.847532, 0, 0.9986781045, 0.01410400148, -0.0006980000731, -0.04942300517, 0.9988133229,
    0.009485003066, -0.04756001538, -0.004475001447, 1, 0, 0, 0, 0.9649395871, 0.02436898957,
    -0.05755497537, 0.2549218909, -0.249116, 0.9993661511, 0.009952001505, 0.03265400494,
    0.01009800153, 0.9854981188, -0.06440700776, 0.09324301124, -0.1262970152, 0.170571,
    0.9927545808, -0.02090099117, 0.08882396249, -0.07817796699, -0.391532, 0.9828788495,
    0.1013909845, -0.05515999155, 0.143618978, 0.9659421276, 0.1884590249, -0.1422460188,
    0.105854014, 0.581348
]

startVel = [
    1.235314324, -0.008525509087, 0.1515293946, -1.161516553, 0.1866449799, -0.1050802848, 0,
    0.935706195, 0.08277326387, 0.3002461862, 0, 0, 0, 0, 0, 1.114409628, 0.3618553952,
    -0.4505575061, 0, -1.725374735, -0.5052852598, -0.8555179722, -0.2221173515, 0, -0.1837617357,
    0.00171895706, 0.03912837591, 0, 0.147945294, 1.837653345, 0.1534535548, 1.491385941, 0,
    -4.632454387, -0.9111172777, -1.300648184, -1.345694622, 0, -1.084238535, 0.1313680236,
    -0.7236998534, 0, -0.5278312973
]

p.resetBasePositionAndOrientation(humanoid, startLocations[0], [0, 0, 0, 1])
p.resetBasePositionAndOrientation(humanoid2, startLocations[1], [0, 0, 0, 1])
p.resetBasePositionAndOrientation(humanoid3, startLocations[2], [0, 0, 0, 1])
p.resetBasePositionAndOrientation(humanoid4, startLocations[3], [0, 0, 0, 1])
p.resetBasePositionAndOrientation(humanoid5, startLocations[4], [0, 0, 0, 1])

index0 = 7
for j in range(p.getNumJoints(humanoid)):
  ji = p.getJointInfo(humanoid, j)
  targetPosition = [0]
  jointType = ji[2]
  if (jointType == p.JOINT_SPHERICAL):
    targetPosition = [
        startPose[index0 + 1], startPose[index0 + 2], startPose[index0 + 3], startPose[index0 + 0]
    ]
    targetVel = [startVel[index0 + 0], startVel[index0 + 1], startVel[index0 + 2]]
    index0 += 4
    print("spherical position: ", targetPosition)
    print("spherical velocity: ", targetVel)
    p.resetJointStateMultiDof(humanoid, j, targetValue=targetPosition, targetVelocity=targetVel)
    p.resetJointStateMultiDof(humanoid5, j, targetValue=targetPosition, targetVelocity=targetVel)
    p.resetJointStateMultiDof(humanoid2, j, targetValue=targetPosition, targetVelocity=targetVel)
  if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
    targetPosition = [startPose[index0]]
    targetVel = [startVel[index0]]
    index0 += 1
    print("revolute:", targetPosition)
    print("revolute	velocity:", targetVel)
    p.resetJointStateMultiDof(humanoid, j, targetValue=targetPosition, targetVelocity=targetVel)
    p.resetJointStateMultiDof(humanoid5, j, targetValue=targetPosition, targetVelocity=targetVel)
    p.resetJointStateMultiDof(humanoid2, j, targetValue=targetPosition, targetVelocity=targetVel)

for j in range(p.getNumJoints(humanoid)):
  ji = p.getJointInfo(humanoid, j)
  targetPosition = [0]
  jointType = ji[2]
  if (jointType == p.JOINT_SPHERICAL):
    targetPosition = [0, 0, 0, 1]
    p.setJointMotorControlMultiDof(humanoid,
                                   j,
                                   p.POSITION_CONTROL,
                                   targetPosition,
                                   targetVelocity=[0, 0, 0],
                                   positionGain=0,
                                   velocityGain=1,
                                   force=[0, 0, 0])
    p.setJointMotorControlMultiDof(humanoid5,
                                   j,
                                   p.POSITION_CONTROL,
                                   targetPosition,
                                   targetVelocity=[0, 0, 0],
                                   positionGain=0,
                                   velocityGain=1,
                                   force=[0, 0, 0])
    p.setJointMotorControlMultiDof(humanoid3,
                                   j,
                                   p.POSITION_CONTROL,
                                   targetPosition,
                                   targetVelocity=[0, 0, 0],
                                   positionGain=0,
                                   velocityGain=1,
                                   force=[31, 31, 31])
    p.setJointMotorControlMultiDof(humanoid4,
                                   j,
                                   p.POSITION_CONTROL,
                                   targetPosition,
                                   targetVelocity=[0, 0, 0],
                                   positionGain=0,
                                   velocityGain=1,
                                   force=[1, 1, 1])

  if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
    p.setJointMotorControl2(humanoid, j, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
    p.setJointMotorControl2(humanoid3, j, p.VELOCITY_CONTROL, targetVelocity=0, force=31)
    p.setJointMotorControl2(humanoid4, j, p.VELOCITY_CONTROL, targetVelocity=0, force=10)
    p.setJointMotorControl2(humanoid5, j, p.VELOCITY_CONTROL, targetVelocity=0, force=0)

  #print(ji)
  print("joint[", j, "].type=", jointTypes[ji[2]])
  print("joint[", j, "].name=", ji[1])

jointIds = []
paramIds = []
for j in range(p.getNumJoints(humanoid)):
  #p.changeDynamics(humanoid,j,linearDamping=0,	angularDamping=0)
  p.changeVisualShape(humanoid, j, rgbaColor=[1, 1, 1, 1])
  info = p.getJointInfo(humanoid, j)
  #print(info)
  if (not useMotionCapture):
    jointName = info[1]
    jointType = info[2]
    if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
      jointIds.append(j)
      #paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"),-4,4,0))
      #print("jointName=",jointName, "at ",	j)

p.changeVisualShape(humanoid, 2, rgbaColor=[1, 0, 0, 1])
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

#rightShoulder=3
#rightElbow=4
#leftShoulder=6
#leftElbow = 7
#rightHip	=	9
#rightKnee=10
#rightAnkle=11
#leftHip = 12
#leftKnee=13
#leftAnkle=14

import time

kpOrg = [
    0, 0, 0, 0, 0, 0, 0, 1000, 1000, 1000, 1000, 100, 100, 100, 100, 500, 500, 500, 500, 500, 400,
    400, 400, 400, 400, 400, 400, 400, 300, 500, 500, 500, 500, 500, 400, 400, 400, 400, 400, 400,
    400, 400, 300
]
kdOrg = [
    0, 0, 0, 0, 0, 0, 0, 100, 100, 100, 100, 10, 10, 10, 10, 50, 50, 50, 50, 50, 40, 40, 40, 40,
    40, 40, 40, 40, 30, 50, 50, 50, 50, 50, 40, 40, 40, 40, 40, 40, 40, 40, 30
]

once = True
p.getCameraImage(320, 200)

while (p.isConnected()):

  if useGUI:
    erp = p.readUserDebugParameter(erpId)
    kpMotor = p.readUserDebugParameter(kpMotorId)
    maxForce = p.readUserDebugParameter(forceMotorId)
    frameReal = p.readUserDebugParameter(frameId)
  else:
    erp = 0.2
    kpMotor = 0.2
    maxForce = 1000
    frameReal = 0

  kp = kpMotor

  frame = int(frameReal)
  frameNext = frame + 1
  if (frameNext >= numFrames):
    frameNext = frame

  frameFraction = frameReal - frame
  #print("frameFraction=",frameFraction)
  #print("frame=",frame)
  #print("frameNext=", frameNext)

  #getQuaternionSlerp

  frameData = motion_dict['Frames'][frame]
  frameDataNext = motion_dict['Frames'][frameNext]

  #print("duration=",frameData[0])
  #print(pos=[frameData])

  basePos1Start = [frameData[1], frameData[2], frameData[3]]
  basePos1End = [frameDataNext[1], frameDataNext[2], frameDataNext[3]]
  basePos1 = [
      basePos1Start[0] + frameFraction * (basePos1End[0] - basePos1Start[0]),
      basePos1Start[1] + frameFraction * (basePos1End[1] - basePos1Start[1]),
      basePos1Start[2] + frameFraction * (basePos1End[2] - basePos1Start[2])
  ]
  baseOrn1Start = [frameData[5], frameData[6], frameData[7], frameData[4]]
  baseOrn1Next = [frameDataNext[5], frameDataNext[6], frameDataNext[7], frameDataNext[4]]
  baseOrn1 = p.getQuaternionSlerp(baseOrn1Start, baseOrn1Next, frameFraction)
  #pre-rotate	to make	z-up
  if (useZUp):
    y2zPos = [0, 0, 0.0]
    y2zOrn = p.getQuaternionFromEuler([1.57, 0, 0])
    basePos, baseOrn = p.multiplyTransforms(y2zPos, y2zOrn, basePos1, baseOrn1)
    p.resetBasePositionAndOrientation(humanoid, basePos, baseOrn)

    y2zPos = [0, 2, 0.0]
    y2zOrn = p.getQuaternionFromEuler([1.57, 0, 0])
    basePos, baseOrn = p.multiplyTransforms(y2zPos, y2zOrn, basePos1, baseOrn1)
    p.resetBasePositionAndOrientation(humanoid2, basePos, baseOrn)

  chestRotStart = [frameData[9], frameData[10], frameData[11], frameData[8]]
  chestRotEnd = [frameDataNext[9], frameDataNext[10], frameDataNext[11], frameDataNext[8]]
  chestRot = p.getQuaternionSlerp(chestRotStart, chestRotEnd, frameFraction)

  neckRotStart = [frameData[13], frameData[14], frameData[15], frameData[12]]
  neckRotEnd = [frameDataNext[13], frameDataNext[14], frameDataNext[15], frameDataNext[12]]
  neckRot = p.getQuaternionSlerp(neckRotStart, neckRotEnd, frameFraction)

  rightHipRotStart = [frameData[17], frameData[18], frameData[19], frameData[16]]
  rightHipRotEnd = [frameDataNext[17], frameDataNext[18], frameDataNext[19], frameDataNext[16]]
  rightHipRot = p.getQuaternionSlerp(rightHipRotStart, rightHipRotEnd, frameFraction)

  rightKneeRotStart = [frameData[20]]
  rightKneeRotEnd = [frameDataNext[20]]
  rightKneeRot = [
      rightKneeRotStart[0] + frameFraction * (rightKneeRotEnd[0] - rightKneeRotStart[0])
  ]

  rightAnkleRotStart = [frameData[22], frameData[23], frameData[24], frameData[21]]
  rightAnkleRotEnd = [frameDataNext[22], frameDataNext[23], frameDataNext[24], frameDataNext[21]]
  rightAnkleRot = p.getQuaternionSlerp(rightAnkleRotStart, rightAnkleRotEnd, frameFraction)

  rightShoulderRotStart = [frameData[26], frameData[27], frameData[28], frameData[25]]
  rightShoulderRotEnd = [
      frameDataNext[26], frameDataNext[27], frameDataNext[28], frameDataNext[25]
  ]
  rightShoulderRot = p.getQuaternionSlerp(rightShoulderRotStart, rightShoulderRotEnd,
                                          frameFraction)

  rightElbowRotStart = [frameData[29]]
  rightElbowRotEnd = [frameDataNext[29]]
  rightElbowRot = [
      rightElbowRotStart[0] + frameFraction * (rightElbowRotEnd[0] - rightElbowRotStart[0])
  ]

  leftHipRotStart = [frameData[31], frameData[32], frameData[33], frameData[30]]
  leftHipRotEnd = [frameDataNext[31], frameDataNext[32], frameDataNext[33], frameDataNext[30]]
  leftHipRot = p.getQuaternionSlerp(leftHipRotStart, leftHipRotEnd, frameFraction)

  leftKneeRotStart = [frameData[34]]
  leftKneeRotEnd = [frameDataNext[34]]
  leftKneeRot = [leftKneeRotStart[0] + frameFraction * (leftKneeRotEnd[0] - leftKneeRotStart[0])]

  leftAnkleRotStart = [frameData[36], frameData[37], frameData[38], frameData[35]]
  leftAnkleRotEnd = [frameDataNext[36], frameDataNext[37], frameDataNext[38], frameDataNext[35]]
  leftAnkleRot = p.getQuaternionSlerp(leftAnkleRotStart, leftAnkleRotEnd, frameFraction)

  leftShoulderRotStart = [frameData[40], frameData[41], frameData[42], frameData[39]]
  leftShoulderRotEnd = [frameDataNext[40], frameDataNext[41], frameDataNext[42], frameDataNext[39]]
  leftShoulderRot = p.getQuaternionSlerp(leftShoulderRotStart, leftShoulderRotEnd, frameFraction)
  leftElbowRotStart = [frameData[43]]
  leftElbowRotEnd = [frameDataNext[43]]
  leftElbowRot = [
      leftElbowRotStart[0] + frameFraction * (leftElbowRotEnd[0] - leftElbowRotStart[0])
  ]

  if (0):  #if (once):
    p.resetJointStateMultiDof(humanoid, chest, chestRot)
    p.resetJointStateMultiDof(humanoid, neck, neckRot)
    p.resetJointStateMultiDof(humanoid, rightHip, rightHipRot)
    p.resetJointStateMultiDof(humanoid, rightKnee, rightKneeRot)
    p.resetJointStateMultiDof(humanoid, rightAnkle, rightAnkleRot)
    p.resetJointStateMultiDof(humanoid, rightShoulder, rightShoulderRot)
    p.resetJointStateMultiDof(humanoid, rightElbow, rightElbowRot)
    p.resetJointStateMultiDof(humanoid, leftHip, leftHipRot)
    p.resetJointStateMultiDof(humanoid, leftKnee, leftKneeRot)
    p.resetJointStateMultiDof(humanoid, leftAnkle, leftAnkleRot)
    p.resetJointStateMultiDof(humanoid, leftShoulder, leftShoulderRot)
    p.resetJointStateMultiDof(humanoid, leftElbow, leftElbowRot)
  once = False
  #print("chestRot=",chestRot)
  p.setGravity(0, 0, -10)

  kp = kpMotor
  if (useExplicitPD):
    jointDofCounts = [4, 4, 4, 1, 4, 4, 1, 4, 1, 4, 4, 1]
    #[x,y,z] base	position and [x,y,z,w] base	orientation!
    totalDofs = 7
    for dof in jointDofCounts:
      totalDofs += dof

    jointIndicesAll = [
        chest, neck, rightHip, rightKnee, rightAnkle, rightShoulder, rightElbow, leftHip, leftKnee,
        leftAnkle, leftShoulder, leftElbow
    ]
    basePos, baseOrn = p.getBasePositionAndOrientation(humanoid)
    pose = [
        basePos[0], basePos[1], basePos[2], baseOrn[0], baseOrn[1], baseOrn[2], baseOrn[3],
        chestRot[0], chestRot[1], chestRot[2], chestRot[3], neckRot[0], neckRot[1], neckRot[2],
        neckRot[3], rightHipRot[0], rightHipRot[1], rightHipRot[2], rightHipRot[3],
        rightKneeRot[0], rightAnkleRot[0], rightAnkleRot[1], rightAnkleRot[2], rightAnkleRot[3],
        rightShoulderRot[0], rightShoulderRot[1], rightShoulderRot[2], rightShoulderRot[3],
        rightElbowRot[0], leftHipRot[0], leftHipRot[1], leftHipRot[2], leftHipRot[3],
        leftKneeRot[0], leftAnkleRot[0], leftAnkleRot[1], leftAnkleRot[2], leftAnkleRot[3],
        leftShoulderRot[0], leftShoulderRot[1], leftShoulderRot[2], leftShoulderRot[3],
        leftElbowRot[0]
    ]

    #print("pose=")
    #for po	in pose:
    #	print(po)
    
    
    taus = stablePD.computePD(bodyUniqueId=humanoid5,
                                jointIndices=jointIndicesAll,
                                desiredPositions=pose,
                                desiredVelocities=[0] * totalDofs,
                                kps=kpOrg,
                                kds=kdOrg,
                                maxForces=[maxForce] * totalDofs,
                                timeStep=timeStep)
    
    indices = [chest, neck, rightHip, rightKnee,
              rightAnkle, rightShoulder, rightElbow,
               leftHip, leftKnee, leftAnkle,
               leftShoulder, leftElbow]
    targetPositions = [chestRot,neckRot,rightHipRot, rightKneeRot,
                      rightAnkleRot, rightShoulderRot, rightElbowRot,
                      leftHipRot, leftKneeRot, leftAnkleRot,
                      leftShoulderRot, leftElbowRot]
    maxForces = [  [maxForce,maxForce,maxForce], [maxForce,maxForce,maxForce],[maxForce,maxForce,maxForce],[maxForce],
                  [maxForce,maxForce,maxForce],[maxForce,maxForce,maxForce],[maxForce],
                  [maxForce,maxForce,maxForce], [maxForce], [maxForce,maxForce,maxForce],
                  [maxForce,maxForce,maxForce], [maxForce]]
    
   
    kps = [1000]*12
    kds = [100]*12
    
                      
    p.setJointMotorControlMultiDofArray(humanoid,
                                   indices,
                                   p.STABLE_PD_CONTROL,
                                   targetPositions=targetPositions,
                                   positionGains=kps,
                                   velocityGains=kds,
                                   forces=maxForces)

    taus3 = explicitPD.computePD(bodyUniqueId=humanoid3,
                                 jointIndices=jointIndicesAll,
                                 desiredPositions=pose,
                                 desiredVelocities=[0] * totalDofs,
                                 kps=kpOrg,
                                 kds=kdOrg,
                                 maxForces=[maxForce * 0.05] * totalDofs,
                                 timeStep=timeStep)

    #taus=[0]*43
    dofIndex = 7
    for index in range(len(jointIndicesAll)):
      jointIndex = jointIndicesAll[index]
      if jointDofCounts[index] == 4:
        
        p.setJointMotorControlMultiDof(
              humanoid5,
            jointIndex,
            p.TORQUE_CONTROL,
            force=[taus[dofIndex + 0], taus[dofIndex + 1], taus[dofIndex + 2]])
        p.setJointMotorControlMultiDof(
            humanoid3,
            jointIndex,
            p.TORQUE_CONTROL,
            force=[taus3[dofIndex + 0], taus3[dofIndex + 1], taus3[dofIndex + 2]])

      if jointDofCounts[index] == 1:

        
        p.setJointMotorControlMultiDof(humanoid5,
                                       jointIndex,
                                       controlMode=p.TORQUE_CONTROL,
                                       force=[taus[dofIndex]])
        p.setJointMotorControlMultiDof(humanoid3,
                                       jointIndex,
                                       controlMode=p.TORQUE_CONTROL,
                                       force=[taus3[dofIndex]])

      dofIndex += jointDofCounts[index]

    #print("len(taus)=",len(taus))
    #print("taus=",taus)

  p.setJointMotorControlMultiDof(humanoid2,
                                 chest,
                                 p.POSITION_CONTROL,
                                 targetPosition=chestRot,
                                 positionGain=kp,
                                 force=[maxForce])
  p.setJointMotorControlMultiDof(humanoid2,
                                 neck,
                                 p.POSITION_CONTROL,
                                 targetPosition=neckRot,
                                 positionGain=kp,
                                 force=[maxForce])
  p.setJointMotorControlMultiDof(humanoid2,
                                 rightHip,
                                 p.POSITION_CONTROL,
                                 targetPosition=rightHipRot,
                                 positionGain=kp,
                                 force=[maxForce])
  p.setJointMotorControlMultiDof(humanoid2,
                                 rightKnee,
                                 p.POSITION_CONTROL,
                                 targetPosition=rightKneeRot,
                                 positionGain=kp,
                                 force=[maxForce])
  p.setJointMotorControlMultiDof(humanoid2,
                                 rightAnkle,
                                 p.POSITION_CONTROL,
                                 targetPosition=rightAnkleRot,
                                 positionGain=kp,
                                 force=[maxForce])
  p.setJointMotorControlMultiDof(humanoid2,
                                 rightShoulder,
                                 p.POSITION_CONTROL,
                                 targetPosition=rightShoulderRot,
                                 positionGain=kp,
                                 force=[maxForce])
  p.setJointMotorControlMultiDof(humanoid2,
                                 rightElbow,
                                 p.POSITION_CONTROL,
                                 targetPosition=rightElbowRot,
                                 positionGain=kp,
                                 force=[maxForce])
  p.setJointMotorControlMultiDof(humanoid2,
                                 leftHip,
                                 p.POSITION_CONTROL,
                                 targetPosition=leftHipRot,
                                 positionGain=kp,
                                 force=[maxForce])
  p.setJointMotorControlMultiDof(humanoid2,
                                 leftKnee,
                                 p.POSITION_CONTROL,
                                 targetPosition=leftKneeRot,
                                 positionGain=kp,
                                 force=[maxForce])
  p.setJointMotorControlMultiDof(humanoid2,
                                 leftAnkle,
                                 p.POSITION_CONTROL,
                                 targetPosition=leftAnkleRot,
                                 positionGain=kp,
                                 force=[maxForce])
  p.setJointMotorControlMultiDof(humanoid2,
                                 leftShoulder,
                                 p.POSITION_CONTROL,
                                 targetPosition=leftShoulderRot,
                                 positionGain=kp,
                                 force=[maxForce])
  p.setJointMotorControlMultiDof(humanoid2,
                                 leftElbow,
                                 p.POSITION_CONTROL,
                                 targetPosition=leftElbowRot,
                                 positionGain=kp,
                                 force=[maxForce])

  kinematicHumanoid4 = True
  if (kinematicHumanoid4):
    p.resetJointStateMultiDof(humanoid4, chest, chestRot)
    p.resetJointStateMultiDof(humanoid4, neck, neckRot)
    p.resetJointStateMultiDof(humanoid4, rightHip, rightHipRot)
    p.resetJointStateMultiDof(humanoid4, rightKnee, rightKneeRot)
    p.resetJointStateMultiDof(humanoid4, rightAnkle, rightAnkleRot)
    p.resetJointStateMultiDof(humanoid4, rightShoulder, rightShoulderRot)
    p.resetJointStateMultiDof(humanoid4, rightElbow, rightElbowRot)
    p.resetJointStateMultiDof(humanoid4, leftHip, leftHipRot)
    p.resetJointStateMultiDof(humanoid4, leftKnee, leftKneeRot)
    p.resetJointStateMultiDof(humanoid4, leftAnkle, leftAnkleRot)
    p.resetJointStateMultiDof(humanoid4, leftShoulder, leftShoulderRot)
    p.resetJointStateMultiDof(humanoid4, leftElbow, leftElbowRot)
  p.stepSimulation()

  if showJointMotorTorques:
    for j in range(p.getNumJoints(humanoid2)):
      jointState = p.getJointStateMultiDof(humanoid2, j)
      print("jointStateMultiDof[", j, "].pos=", jointState[0])
      print("jointStateMultiDof[", j, "].vel=", jointState[1])
      print("jointStateMultiDof[", j, "].jointForces=", jointState[3])
  time.sleep(timeStep)
