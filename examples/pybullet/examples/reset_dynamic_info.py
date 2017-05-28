import pybullet as p
import time
import math

p.connect(p.GUI)
planeId = p.loadURDF(fileName="plane.urdf",baseOrientation=[0.25882,0,0,0.96593])
p.loadURDF(fileName="cube.urdf",baseOrientation=[0.25882,0,0,0.96593],basePosition=[0,0,2])
cubeId = p.loadURDF(fileName="cube.urdf",baseOrientation=[0,0,0,1],basePosition=[0,0,4])
#p.changeDynamics(bodyUniqueId=2,linkIndex=-1,mass=0.1)
p.changeDynamics(bodyUniqueId=2,linkIndex=-1,mass=100.0)
p.setGravity(0,0,-10)
p.setRealTimeSimulation(0)
t=0
while 1:
	t=t+1
	if t > 400:
		p.changeDynamics(bodyUniqueId=0,linkIndex=-1,lateralFriction=0.01)
	mass1,frictionCoeff1=p.getDynamicsInfo(bodyUniqueId=planeId,linkIndex=-1)
	mass2,frictionCoeff2=p.getDynamicsInfo(bodyUniqueId=cubeId,linkIndex=-1)
	print mass1,frictionCoeff1
	print mass2,frictionCoeff2
	time.sleep(.01)
	p.stepSimulation()
