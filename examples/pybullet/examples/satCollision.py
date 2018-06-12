import pybullet as p
import time 

p.connect(p.GUI)
p.setGravity(0,0,-10)
p.setPhysicsEngineParameter(enableSAT=1)
p.loadURDF("cube_concave.urdf",[0,0,-25], globalScaling=50, useFixedBase=True)
p.loadURDF("cube.urdf",[0,0,1], globalScaling=1)
p.loadURDF("duck_vhacd.urdf",[1,0,1], globalScaling=1)

while (p.isConnected()):
	p.stepSimulation()
	time.sleep(1./240.)
	