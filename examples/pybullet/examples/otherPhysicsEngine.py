import pybullet as p
import time

#p.connect(p.DIRECT)
#p.connect(p.DART)
p.connect(p.MuJoCo)

#p.connect(p.GUI)
bodies = p.loadMJCF("mjcf/capsule.xml")
print("bodies=",bodies)

numBodies = p.getNumBodies()
print("numBodies=",numBodies)
for i in range (numBodies):
	print("bodyInfo[",i,"]=",p.getBodyInfo(i))
	
p.setGravity(0,0,-10)
timeStep = 1./240.
p.setPhysicsEngineParameter(fixedTimeStep=timeStep)

#while (p.isConnected()):
for i in range (1000):
	p.stepSimulation()

	for b in bodies:
		pos,orn=p.getBasePositionAndOrientation(b)
		print("pos[",b,"]=",pos)
		print("orn[",b,"]=",orn)
		linvel,angvel=p.getBaseVelocity(b)
		print("linvel[",b,"]=",linvel)
		print("angvel[",b,"]=",angvel)
	time.sleep(timeStep)
