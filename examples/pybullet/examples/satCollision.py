import pybullet as p
import time 

p.connect(p.GUI)
p.setGravity(0,0,-10)
p.setPhysicsEngineParameter(enableSAT=1)
p.loadURDF("cube_concave.urdf",[0,0,-25], globalScaling=50, useFixedBase=True, flags=p.URDF_INITIALIZE_SAT_FEATURES)
p.loadURDF("cube.urdf",[0,0,1], globalScaling=1,flags=p.URDF_INITIALIZE_SAT_FEATURES)
p.loadURDF("duck_vhacd.urdf",[1,0,1], globalScaling=1,flags=p.URDF_INITIALIZE_SAT_FEATURES)

while (p.isConnected()):
	p.stepSimulation()
	pts = p.getContactPoints()
	#print("num contacts = ", len(pts))
	time.sleep(1./240.)
	