import pybullet as p
import time

p.connect(p.PhysX)
p.loadPlugin("eglRendererPlugin")

p.loadURDF("plane.urdf")
for i in range (50):
	p.loadURDF("r2d2.urdf",[0,0,1+i*2])
p.setGravity(0,0,-10)

while (1):
	p.stepSimulation()
	time.sleep(1./240.)