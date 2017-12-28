import pybullet as p
import math, time

p.connect(p.GUI, options="--width=1024 --height=768")

p.loadURDF("plane.urdf")
for i in range (10):
	cube = p.loadURDF("cube_small.urdf",0,i*0.01,i*0.5)
p.setGravity(0,0,-10)


for i in range (500):
	p.stepSimulation()

for i in range (10):
	print("pos[",i,"]=",p.getBasePositionAndOrientation(i))

#saveState = 0

#if saveState:
#	for i in range (500):
#		p.stepSimulation()
#	p.saveBullet("state.bullet")
#else:
#	p.restoreState(fileName="state.bullet")


while (p.getConnectionInfo()["isConnected"]):
	time.sleep(1)