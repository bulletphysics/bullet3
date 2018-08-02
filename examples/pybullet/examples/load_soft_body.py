import pybullet as p
from time import sleep

physicsClient = p.connect(p.GUI)

p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
bunnyId = p.loadSoftBody("bunny.obj")
p.loadURDF("cube_small.urdf",[1,0,1])
useRealTimeSimulation = 1

if (useRealTimeSimulation):
	p.setRealTimeSimulation(1)

while p.isConnected():
	p.setGravity(0,0,-10)
	if (useRealTimeSimulation):

		sleep(0.01) # Time in seconds.
		#p.getCameraImage(320,200,renderer=p.ER_BULLET_HARDWARE_OPENGL )
	else:
		p.stepSimulation()
