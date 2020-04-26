#script to track a robot with a VR controller attached to it.

import time
import pybullet as p
import pybullet_data


#first try to connect to shared memory (VR), if it fails use local GUI
c = p.connect(p.SHARED_MEMORY)
if (c < 0):
  c = p.connect(p.GUI)
p.resetSimulation()

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)
print(c)
if (c < 0):
  p.connect(p.GUI)

#load the MuJoCo MJCF hand
minitaur = p.loadURDF("quadruped/minitaur.urdf")
robot_cid = -1

POSITION = 1
ORIENTATION = 2
BUTTONS = 6

p.setRealTimeSimulation(1)

controllerId = -1

while True:
  events = p.getVREvents()
  for e in (events):
    #print (e[BUTTONS])
    if (e[BUTTONS][33] & p.VR_BUTTON_WAS_TRIGGERED):
      if (controllerId >= 0):
        controllerId = -1
      else:
        controllerId = e[0]
    if (e[0] == controllerId):
      if (robot_cid >= 0):
        p.changeConstraint(robot_cid, e[POSITION], e[ORIENTATION], maxForce=5000)
    if (e[BUTTONS][32] & p.VR_BUTTON_WAS_TRIGGERED):
      if (robot_cid >= 0):
        p.removeConstraint(robot_cid)

      #q = [0,0,0,1]
      euler = p.getEulerFromQuaternion(e[ORIENTATION])
      q = p.getQuaternionFromEuler([euler[0], euler[1], 0])  #-euler[0],-euler[1],-euler[2]])
      robot_cid = p.createConstraint(minitaur, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0.1, 0, 0],
                                     e[POSITION], q, e[ORIENTATION])
