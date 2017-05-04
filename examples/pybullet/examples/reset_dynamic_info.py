import pybullet as p
import time
import math

p.connect(p.GUI)

p.loadURDF(fileName="plane.urdf",baseOrientation=[0.25882,0,0,0.96593])
p.loadURDF(fileName="cube.urdf",baseOrientation=[0.25882,0,0,0.96593],basePosition=[0,0,2])
p.loadURDF(fileName="cube.urdf",baseOrientation=[0,0,0,1],basePosition=[0,0,4])
p.resetDynamicInfo(bodyUniqueId=2,linkIndex=-1,mass=0.1)
#p.resetDynamicInfo(bodyUniqueId=2,linkIndex=-1,mass=100.0)
p.setGravity(0,0,-10)
p.setRealTimeSimulation(0)
t=0
while 1:
	t=t+1
	if t > 400:
		p.resetDynamicInfo(bodyUniqueId=0,linkIndex=-1,lateralFriction=0.01)
	time.sleep(.01)
	p.stepSimulation()
