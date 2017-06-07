#mimic joint currently only works for 'maximal coordinate' rigid bodies
#one way to use it would be to attach maximal coordinate rigid bodies to multibody links using
#fixed constraints

import pybullet as p
import time
p.connect(p.GUI)
wheelA = p.loadURDF("wheel.urdf",[0,0,0],useMaximalCoordinates=1)
wheelB = p.loadURDF("wheel.urdf",[0,0,1],useMaximalCoordinates=1)
c = p.createConstraint(wheelA,-1,wheelB,-1,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
p.changeConstraint(c,gearRatio=-0.1)
p.setRealTimeSimulation(1)
while(1):
	time.sleep(0.01)
p.removeConstraint(c)
	