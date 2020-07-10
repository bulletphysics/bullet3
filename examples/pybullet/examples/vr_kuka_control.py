## Assume you have run vr_kuka_setup and have default scene set up
# Require p.setInternalSimFlags(0) in kuka_setup
import pybullet as p
import math
# import numpy as np
import pybullet_data


p.connect(p.SHARED_MEMORY)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

kuka = 3
kuka_gripper = 7
POSITION = 1
ORIENTATION = 2
ANALOG = 3
BUTTONS = 6

THRESHOLD = .5
LOWER_LIMITS = [-.967, -2.0, -2.96, 0.19, -2.96, -2.09, -3.05]
UPPER_LIMITS = [.96, 2.0, 2.96, 2.29, 2.96, 2.09, 3.05]
JOINT_RANGE = [5.8, 4, 5.8, 4, 5.8, 4, 6]
REST_POSE = [0, 0, 0, math.pi / 2, 0, -math.pi * 0.66, 0]
JOINT_DAMP = [0.1] * 10
REST_JOINT_POS = [-0., -0., 0., 1.570793, 0., -1.036725, 0.000001]
MAX_FORCE = 500
KUKA_GRIPPER_REST_POS = [0., -0.011130, -0.206421, 0.205143, -0.009999, 0., -0.010055, 0.]
KUKA_GRIPPER_CLOZ_POS = [
    0.0, -0.047564246423083795, 0.6855956234759611, -0.7479294372303137, 0.05054599996976922, 0.0,
    0.049838105678835724, 0.0
]


def euc_dist(posA, posB):
  dist = 0.
  for i in range(len(posA)):
    dist += (posA[i] - posB[i])**2
  return dist


p.setRealTimeSimulation(1)

controllers = [e[0] for e in p.getVREvents()]

for j in range(p.getNumJoints(kuka_gripper)):
  print(p.getJointInfo(kuka_gripper, j))
while True:

  events = p.getVREvents()
  for e in (events):

    # Only use one controller
    ###########################################
    # This is important: make sure there's only one VR Controller!
    if e[0] == controllers[0]:
      break

    sq_len = euc_dist(p.getLinkState(kuka, 6)[0], e[POSITION])

    # A simplistic version of gripper control
    #@TO-DO: Add slider for the gripper

    #for i in range(p.getNumJoints(kuka_gripper)):
    i = 4
    p.setJointMotorControl2(kuka_gripper,
                            i,
                            p.POSITION_CONTROL,
                            targetPosition=e[ANALOG] * 0.05,
                            force=10)
    i = 6
    p.setJointMotorControl2(kuka_gripper,
                            i,
                            p.POSITION_CONTROL,
                            targetPosition=e[ANALOG] * 0.05,
                            force=10)

    if sq_len < THRESHOLD * THRESHOLD:
      eef_pos = e[POSITION]
      eef_orn = p.getQuaternionFromEuler([0, -math.pi, 0])
      joint_pos = p.calculateInverseKinematics(kuka,
                                               6,
                                               eef_pos,
                                               eef_orn,
                                               lowerLimits=LOWER_LIMITS,
                                               upperLimits=UPPER_LIMITS,
                                               jointRanges=JOINT_RANGE,
                                               restPoses=REST_POSE,
                                               jointDamping=JOINT_DAMP)

      for i in range(len(joint_pos)):
        p.setJointMotorControl2(kuka,
                                i,
                                p.POSITION_CONTROL,
                                targetPosition=joint_pos[i],
                                targetVelocity=0,
                                positionGain=0.15,
                                velocityGain=1.0,
                                force=MAX_FORCE)

    else:
      # Set back to original rest pose
      for jointIndex in range(p.getNumJoints(kuka)):
        p.setJointMotorControl2(kuka, jointIndex, p.POSITION_CONTROL, REST_JOINT_POS[jointIndex],
                                0)
