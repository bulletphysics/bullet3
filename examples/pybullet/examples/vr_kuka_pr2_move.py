#python script with hardcoded values, assumes that you run the vr_kuka_setup.py first

import pybullet as p
import pybullet_data

p.connect(p.SHARED_MEMORY)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

pr2_gripper = 2
pr2_cid = 1

CONTROLLER_ID = 0
POSITION = 1
ORIENTATION = 2
ANALOG = 3
BUTTONS = 6

gripper_max_joint = 0.550569
while True:
  events = p.getVREvents()
  for e in (events):
    if e[CONTROLLER_ID] == 3:  # To make sure we only get the value for one of the remotes
      p.changeConstraint(pr2_cid, e[POSITION], e[ORIENTATION], maxForce=500)
      p.setJointMotorControl2(pr2_gripper,
                              0,
                              controlMode=p.POSITION_CONTROL,
                              targetPosition=gripper_max_joint - e[ANALOG] * gripper_max_joint,
                              force=1.0)
      p.setJointMotorControl2(pr2_gripper,
                              2,
                              controlMode=p.POSITION_CONTROL,
                              targetPosition=gripper_max_joint - e[ANALOG] * gripper_max_joint,
                              force=1.1)
