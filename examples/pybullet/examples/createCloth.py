import pybullet as p
from time import sleep

physicsClient = p.connect(p.GUI)

p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
clothId = p.createCloth(
	[0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1], # Corners (each corner represented as xyz pos)
	[30, 30], # Resolution
	1 + 2 + 4 + 8  # Binary mask to fix corners
)
while 1:

	sleep(0.01) 
	p.stepSimulation()
