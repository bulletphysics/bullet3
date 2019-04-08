import pybullet as p
import time
import math

pi = 3.14159264
limitVal = 2 * pi
legpos = 3. / 4. * pi
legposS = 0
legposSright = 0  #-0.3
legposSleft = 0  #0.3

defaultERP = 0.4
maxMotorForce = 5000
maxGearForce = 10000
jointFriction = 0.1

p.connect(p.GUI)

amplitudeId = p.addUserDebugParameter("amplitude", 0, 3.14, 0.5)
timeScaleId = p.addUserDebugParameter("timeScale", 0, 10, 1)

jointTypeNames = {}
jointTypeNames[p.JOINT_REVOLUTE] = "JOINT_REVOLUTE"
jointTypeNames[p.JOINT_FIXED] = "JOINT_FIXED"
p.setPhysicsEngineParameter(numSolverIterations=100)
p.loadURDF("plane_transparent.urdf", useMaximalCoordinates=True)
#disable rendering during creation.
#p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
p.configureDebugVisualizer(p.COV_ENABLE_PLANAR_REFLECTION, 1)

jointNamesToIndex = {}

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
vision = p.loadURDF("vision60.urdf", [0, 0, 0.4], useFixedBase=False)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

for j in range(p.getNumJoints(vision)):
  jointInfo = p.getJointInfo(vision, j)
  jointInfoName = jointInfo[1].decode("utf-8")
  print("joint ", j, " = ", jointInfoName, "type=", jointTypeNames[jointInfo[2]])
  jointNamesToIndex[jointInfoName] = j
  #print("jointNamesToIndex[..]=",jointNamesToIndex[jointInfoName])
  p.setJointMotorControl2(vision, j, p.VELOCITY_CONTROL, targetVelocity=0, force=jointFriction)

chassis_right_center = jointNamesToIndex['chassis_right_center']
motor_front_rightR_joint = jointNamesToIndex['motor_front_rightR_joint']
motor_front_rightS_joint = jointNamesToIndex['motor_front_rightS_joint']
hip_front_rightR_joint = jointNamesToIndex['hip_front_rightR_joint']
knee_front_rightR_joint = jointNamesToIndex['knee_front_rightR_joint']
motor_front_rightL_joint = jointNamesToIndex['motor_front_rightL_joint']
motor_back_rightR_joint = jointNamesToIndex['motor_back_rightR_joint']
motor_back_rightS_joint = jointNamesToIndex['motor_back_rightS_joint']
hip_back_rightR_joint = jointNamesToIndex['hip_back_rightR_joint']
knee_back_rightR_joint = jointNamesToIndex['knee_back_rightR_joint']
motor_back_rightL_joint = jointNamesToIndex['motor_back_rightL_joint']
chassis_left_center = jointNamesToIndex['chassis_left_center']
motor_front_leftL_joint = jointNamesToIndex['motor_front_leftL_joint']
motor_front_leftS_joint = jointNamesToIndex['motor_front_leftS_joint']
hip_front_leftL_joint = jointNamesToIndex['hip_front_leftL_joint']
knee_front_leftL_joint = jointNamesToIndex['knee_front_leftL_joint']
motor_front_leftR_joint = jointNamesToIndex['motor_front_leftR_joint']
motor_back_leftL_joint = jointNamesToIndex['motor_back_leftL_joint']
hip_back_leftL_joint = jointNamesToIndex['hip_back_leftL_joint']
knee_back_leftL_joint = jointNamesToIndex['knee_back_leftL_joint']
motor_back_leftR_joint = jointNamesToIndex['motor_back_leftR_joint']
motor_back_leftS_joint = jointNamesToIndex['motor_back_leftS_joint']

motA_rf_Id = p.addUserDebugParameter("motA_rf", -limitVal, limitVal, legpos)
motB_rf_Id = p.addUserDebugParameter("motB_rf", -limitVal, limitVal, legpos)
motC_rf_Id = p.addUserDebugParameter("motC_rf", -limitVal, limitVal, legposSright)
erp_rf_Id = p.addUserDebugParameter("erp_rf", 0, 1, defaultERP)
relPosTarget_rf_Id = p.addUserDebugParameter("relPosTarget_rf", -limitVal, limitVal, 0)

motA_lf_Id = p.addUserDebugParameter("motA_lf", -limitVal, limitVal, -legpos)
motB_lf_Id = p.addUserDebugParameter("motB_lf", -limitVal, limitVal, -legpos)
motC_lf_Id = p.addUserDebugParameter("motC_lf", -limitVal, limitVal, legposSleft)

erp_lf_Id = p.addUserDebugParameter("erp_lf", 0, 1, defaultERP)
relPosTarget_lf_Id = p.addUserDebugParameter("relPosTarget_lf", -limitVal, limitVal, 0)

motA_rb_Id = p.addUserDebugParameter("motA_rb", -limitVal, limitVal, legpos)
motB_rb_Id = p.addUserDebugParameter("motB_rb", -limitVal, limitVal, legpos)
motC_rb_Id = p.addUserDebugParameter("motC_rb", -limitVal, limitVal, legposSright)

erp_rb_Id = p.addUserDebugParameter("erp_rb", 0, 1, defaultERP)
relPosTarget_rb_Id = p.addUserDebugParameter("relPosTarget_rb", -limitVal, limitVal, 0)

motA_lb_Id = p.addUserDebugParameter("motA_lb", -limitVal, limitVal, -legpos)
motB_lb_Id = p.addUserDebugParameter("motB_lb", -limitVal, limitVal, -legpos)
motC_lb_Id = p.addUserDebugParameter("motC_lb", -limitVal, limitVal, legposSleft)

erp_lb_Id = p.addUserDebugParameter("erp_lb", 0, 1, defaultERP)
relPosTarget_lb_Id = p.addUserDebugParameter("relPosTarget_lb", -limitVal, limitVal, 0)

camTargetPos = [0.25, 0.62, -0.15]
camDist = 2
camYaw = -2
camPitch = -16
p.resetDebugVisualizerCamera(camDist, camYaw, camPitch, camTargetPos)

c_rf = p.createConstraint(vision,
                          knee_front_rightR_joint,
                          vision,
                          motor_front_rightL_joint,
                          jointType=p.JOINT_GEAR,
                          jointAxis=[0, 1, 0],
                          parentFramePosition=[0, 0, 0],
                          childFramePosition=[0, 0, 0])
p.changeConstraint(c_rf, gearRatio=-1, gearAuxLink=motor_front_rightR_joint, maxForce=maxGearForce)

c_lf = p.createConstraint(vision,
                          knee_front_leftL_joint,
                          vision,
                          motor_front_leftR_joint,
                          jointType=p.JOINT_GEAR,
                          jointAxis=[0, 1, 0],
                          parentFramePosition=[0, 0, 0],
                          childFramePosition=[0, 0, 0])
p.changeConstraint(c_lf, gearRatio=-1, gearAuxLink=motor_front_leftL_joint, maxForce=maxGearForce)

c_rb = p.createConstraint(vision,
                          knee_back_rightR_joint,
                          vision,
                          motor_back_rightL_joint,
                          jointType=p.JOINT_GEAR,
                          jointAxis=[0, 1, 0],
                          parentFramePosition=[0, 0, 0],
                          childFramePosition=[0, 0, 0])
p.changeConstraint(c_rb, gearRatio=-1, gearAuxLink=motor_back_rightR_joint, maxForce=maxGearForce)

c_lb = p.createConstraint(vision,
                          knee_back_leftL_joint,
                          vision,
                          motor_back_leftR_joint,
                          jointType=p.JOINT_GEAR,
                          jointAxis=[0, 1, 0],
                          parentFramePosition=[0, 0, 0],
                          childFramePosition=[0, 0, 0])
p.changeConstraint(c_lb, gearRatio=-1, gearAuxLink=motor_back_leftL_joint, maxForce=maxGearForce)

p.setRealTimeSimulation(1)
for i in range(1):
  #while (1):
  motA_rf = p.readUserDebugParameter(motA_rf_Id)
  motB_rf = p.readUserDebugParameter(motB_rf_Id)
  motC_rf = p.readUserDebugParameter(motC_rf_Id)
  erp_rf = p.readUserDebugParameter(erp_rf_Id)
  relPosTarget_rf = p.readUserDebugParameter(relPosTarget_rf_Id)
  #motC_rf
  p.setJointMotorControl2(vision,
                          motor_front_rightR_joint,
                          p.POSITION_CONTROL,
                          targetPosition=motA_rf,
                          force=maxMotorForce)
  p.setJointMotorControl2(vision,
                          motor_front_rightL_joint,
                          p.POSITION_CONTROL,
                          targetPosition=motB_rf,
                          force=maxMotorForce)
  p.setJointMotorControl2(vision,
                          motor_front_rightS_joint,
                          p.POSITION_CONTROL,
                          targetPosition=motC_rf,
                          force=maxMotorForce)
  p.changeConstraint(c_rf,
                     gearRatio=-1,
                     gearAuxLink=motor_front_rightR_joint,
                     erp=erp_rf,
                     relativePositionTarget=relPosTarget_rf,
                     maxForce=maxGearForce)

  motA_lf = p.readUserDebugParameter(motA_lf_Id)
  motB_lf = p.readUserDebugParameter(motB_lf_Id)
  motC_lf = p.readUserDebugParameter(motC_lf_Id)
  erp_lf = p.readUserDebugParameter(erp_lf_Id)
  relPosTarget_lf = p.readUserDebugParameter(relPosTarget_lf_Id)
  p.setJointMotorControl2(vision,
                          motor_front_leftL_joint,
                          p.POSITION_CONTROL,
                          targetPosition=motA_lf,
                          force=maxMotorForce)
  p.setJointMotorControl2(vision,
                          motor_front_leftR_joint,
                          p.POSITION_CONTROL,
                          targetPosition=motB_lf,
                          force=maxMotorForce)
  p.setJointMotorControl2(vision,
                          motor_front_leftS_joint,
                          p.POSITION_CONTROL,
                          targetPosition=motC_lf,
                          force=maxMotorForce)
  p.changeConstraint(c_lf,
                     gearRatio=-1,
                     gearAuxLink=motor_front_leftL_joint,
                     erp=erp_lf,
                     relativePositionTarget=relPosTarget_lf,
                     maxForce=maxGearForce)

  motA_rb = p.readUserDebugParameter(motA_rb_Id)
  motB_rb = p.readUserDebugParameter(motB_rb_Id)
  motC_rb = p.readUserDebugParameter(motC_rb_Id)
  erp_rb = p.readUserDebugParameter(erp_rb_Id)
  relPosTarget_rb = p.readUserDebugParameter(relPosTarget_rb_Id)
  p.setJointMotorControl2(vision,
                          motor_back_rightR_joint,
                          p.POSITION_CONTROL,
                          targetPosition=motA_rb,
                          force=maxMotorForce)
  p.setJointMotorControl2(vision,
                          motor_back_rightL_joint,
                          p.POSITION_CONTROL,
                          targetPosition=motB_rb,
                          force=maxMotorForce)
  p.setJointMotorControl2(vision,
                          motor_back_rightS_joint,
                          p.POSITION_CONTROL,
                          targetPosition=motC_rb,
                          force=maxMotorForce)
  p.changeConstraint(c_rb,
                     gearRatio=-1,
                     gearAuxLink=motor_back_rightR_joint,
                     erp=erp_rb,
                     relativePositionTarget=relPosTarget_rb,
                     maxForce=maxGearForce)

  motA_lb = p.readUserDebugParameter(motA_lb_Id)
  motB_lb = p.readUserDebugParameter(motB_lb_Id)
  motC_lb = p.readUserDebugParameter(motC_lb_Id)
  erp_lb = p.readUserDebugParameter(erp_lb_Id)
  relPosTarget_lb = p.readUserDebugParameter(relPosTarget_lb_Id)
  p.setJointMotorControl2(vision,
                          motor_back_leftL_joint,
                          p.POSITION_CONTROL,
                          targetPosition=motA_lb,
                          force=maxMotorForce)
  p.setJointMotorControl2(vision,
                          motor_back_leftR_joint,
                          p.POSITION_CONTROL,
                          targetPosition=motB_lb,
                          force=maxMotorForce)
  p.setJointMotorControl2(vision,
                          motor_back_leftS_joint,
                          p.POSITION_CONTROL,
                          targetPosition=motC_lb,
                          force=maxMotorForce)
  p.changeConstraint(c_lb,
                     gearRatio=-1,
                     gearAuxLink=motor_back_leftL_joint,
                     erp=erp_lb,
                     relativePositionTarget=relPosTarget_lb,
                     maxForce=maxGearForce)

  p.setGravity(0, 0, -10)
  time.sleep(1. / 240.)
t = 0
prevTime = time.time()
while (1):
  timeScale = p.readUserDebugParameter(timeScaleId)
  amplitude = p.readUserDebugParameter(amplitudeId)
  newTime = time.time()
  dt = (newTime - prevTime) * timeScale
  t = t + dt
  prevTime = newTime

  amp = amplitude
  motA_rf = math.sin(t) * amp + legpos
  motA_rb = math.sin(t) * amp + legpos
  motA_lf = -(math.sin(t) * amp + legpos)
  motA_lb = -(math.sin(t) * amp + legpos)

  motB_rf = math.sin(t) * amp + legpos
  motB_rb = math.sin(t) * amp + legpos
  motB_lf = -(math.sin(t) * amp + legpos)
  motB_lb = -(math.sin(t) * amp + legpos)

  p.setJointMotorControl2(vision,
                          motor_front_rightR_joint,
                          p.POSITION_CONTROL,
                          targetPosition=motA_rf,
                          force=maxMotorForce)
  p.setJointMotorControl2(vision,
                          motor_front_rightL_joint,
                          p.POSITION_CONTROL,
                          targetPosition=motB_rf,
                          force=maxMotorForce)
  p.setJointMotorControl2(vision,
                          motor_front_rightS_joint,
                          p.POSITION_CONTROL,
                          targetPosition=motC_rf,
                          force=maxMotorForce)

  p.setJointMotorControl2(vision,
                          motor_front_leftL_joint,
                          p.POSITION_CONTROL,
                          targetPosition=motA_lf,
                          force=maxMotorForce)
  p.setJointMotorControl2(vision,
                          motor_front_leftR_joint,
                          p.POSITION_CONTROL,
                          targetPosition=motB_lf,
                          force=maxMotorForce)
  p.setJointMotorControl2(vision,
                          motor_front_leftS_joint,
                          p.POSITION_CONTROL,
                          targetPosition=motC_lf,
                          force=maxMotorForce)

  p.setJointMotorControl2(vision,
                          motor_back_rightR_joint,
                          p.POSITION_CONTROL,
                          targetPosition=motA_rb,
                          force=maxMotorForce)
  p.setJointMotorControl2(vision,
                          motor_back_rightL_joint,
                          p.POSITION_CONTROL,
                          targetPosition=motB_rb,
                          force=maxMotorForce)
  p.setJointMotorControl2(vision,
                          motor_back_rightS_joint,
                          p.POSITION_CONTROL,
                          targetPosition=motC_rb,
                          force=maxMotorForce)

  p.setJointMotorControl2(vision,
                          motor_back_leftL_joint,
                          p.POSITION_CONTROL,
                          targetPosition=motA_lb,
                          force=maxMotorForce)
  p.setJointMotorControl2(vision,
                          motor_back_leftR_joint,
                          p.POSITION_CONTROL,
                          targetPosition=motB_lb,
                          force=maxMotorForce)
  p.setJointMotorControl2(vision,
                          motor_back_leftS_joint,
                          p.POSITION_CONTROL,
                          targetPosition=motC_lb,
                          force=maxMotorForce)

  p.setGravity(0, 0, -10)
  time.sleep(1. / 240.)
