import pybullet as p
import pybullet_data as pd
import time

p.connect(p.GUI)
p.setGravity(0,0,-9.8)
p.setAdditionalSearchPath(pd.getDataPath())
floor = p.loadURDF("plane.urdf")
startPos = [0,0,0.5]
robot = p.loadURDF("mini_cheetah/mini_cheetah.urdf",startPos)
numJoints = p.getNumJoints(robot)
p.changeVisualShape(robot,-1,rgbaColor=[1,1,1,1])
for j in range (numJoints):
	p.changeVisualShape(robot,j,rgbaColor=[1,1,1,1])
	force=200
	pos=0
	p.setJointMotorControl2(robot,j,p.POSITION_CONTROL,pos,force=force)
dt = 1./240.
p.setTimeStep(dt)
while (1):
  p.stepSimulation()
  time.sleep(dt)

