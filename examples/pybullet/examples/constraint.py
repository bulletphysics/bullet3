import pybullet as p
import time
import math

import pybullet_data
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf")
cubeId = p.loadURDF("cube_small.urdf", 0, 0, 1)
cubeId2 = p.loadURDF("cube_small.urdf", 0, 0, 1)

p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)

cid = p.createConstraint(cubeId, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 1])
cid2 = p.createConstraint(cubeId2, -1, cubeId, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 1])

#cid2 = p.createConstraint(base_1, -1, base_2, 0, p.JOINT_FIXED, [0, 0, 0], [0, 0, 2], [0, 0, 0])


print(cid)
print(p.getConstraintUniqueId(0))
prev = [0, 0, 1]
a = -math.pi
while 1:
  a = a + 0.01
  if (a > math.pi):
    a = -math.pi
  time.sleep(.01)
  p.setGravity(0, 0, -10)
  pivot = [a, 0, 1]
  pivot2 = [-a, 0, 1]
  orn = p.getQuaternionFromEuler([a, 0, 0])
  p.changeConstraint(cid, pivot, jointChildFrameOrientation=orn, maxForce=50)
  #p.changeConstraint(cid2, pivot2, jointChildFrameOrientation=orn, maxForce=50)


p.removeConstraint(cid)
