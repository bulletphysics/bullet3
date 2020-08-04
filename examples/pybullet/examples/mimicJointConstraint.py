#a mimic joint can act as a gear between two joints
#you can control the gear ratio in magnitude and sign (>0 reverses direction)

import pybullet as p
import time
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", 0, 0, -2)
wheelA = p.loadURDF("differential/diff_ring.urdf", [0, 0, 0])
for i in range(p.getNumJoints(wheelA)):
  print(p.getJointInfo(wheelA, i))
  p.setJointMotorControl2(wheelA, i, p.VELOCITY_CONTROL, targetVelocity=0, force=0)

c = p.createConstraint(wheelA,
                       1,
                       wheelA,
                       3,
                       jointType=p.JOINT_GEAR,
                       jointAxis=[0, 1, 0],
                       parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=1, maxForce=10000)

c = p.createConstraint(wheelA,
                       2,
                       wheelA,
                       4,
                       jointType=p.JOINT_GEAR,
                       jointAxis=[0, 1, 0],
                       parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(wheelA,
                       1,
                       wheelA,
                       4,
                       jointType=p.JOINT_GEAR,
                       jointAxis=[0, 1, 0],
                       parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

p.setRealTimeSimulation(1)
while (1):
  p.setGravity(0, 0, -10)
  time.sleep(0.01)
#p.removeConstraint(c)
