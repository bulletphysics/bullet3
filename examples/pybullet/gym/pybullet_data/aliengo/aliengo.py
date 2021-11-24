import pybullet as p
import time
import pybullet_data as pd
import numpy as np
p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
dt = 1./240.

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
p.loadURDF("plane.urdf")
robot = p.loadURDF("aliengo/aliengo.urdf",[0,0,0.5])
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
p.setGravity(0,0,-9.8)

ALIENGO_DEFAULT_ABDUCTION_ANGLE = 0
ALIENGO_DEFAULT_HIP_ANGLE = 1.2
ALIENGO_DEFAULT_KNEE_ANGLE = -2.0
NUM_LEGS = 4
INIT_MOTOR_ANGLES = np.array([
    ALIENGO_DEFAULT_ABDUCTION_ANGLE,
    ALIENGO_DEFAULT_HIP_ANGLE,
    ALIENGO_DEFAULT_KNEE_ANGLE
] * NUM_LEGS)

MOTOR_NAMES = [
    "FR_hip_joint",
    "FR_upper_joint",
    "FR_lower_joint",
    "FL_hip_joint",
    "FL_upper_joint",
    "FL_lower_joint",
    "RR_hip_joint",
    "RR_upper_joint",
    "RR_lower_joint",
    "RL_hip_joint",
    "RL_upper_joint",
    "RL_lower_joint",
]
motor_ids = []

for j in range (p.getNumJoints(robot)):
  joint_info = p.getJointInfo(robot,j)
  name = joint_info[1].decode('utf-8')
  print("joint_info[1]=",name)
  if name in MOTOR_NAMES:
    motor_ids.append(j)

for index in range (12):
  joint_id = motor_ids[index]
  p.setJointMotorControl2(robot, joint_id, p.POSITION_CONTROL, INIT_MOTOR_ANGLES[index])
  p.resetJointState(robot, joint_id, INIT_MOTOR_ANGLES[index])
  
print("motor_ids=",motor_ids)
while p.isConnected():
  p.stepSimulation()
  time.sleep(dt)


