#you can set the restitution (bouncyness) of an object in the URDF file
#or using changeDynamics

import pybullet as p
import time

cid = p.connect(p.SHARED_MEMORY)
if (cid<0):
	cid = p.connect(p.GUI)
restitutionId = p.addUserDebugParameter("restitution",0,1,1)
restitutionVelocityThresholdId = p.addUserDebugParameter("res. vel. threshold",0,3,0.2)

lateralFrictionId = p.addUserDebugParameter("lateral friction",0,1,0.5)
spinningFrictionId = p.addUserDebugParameter("spinning friction",0,1,0.03)
rollingFrictionId = p.addUserDebugParameter("rolling friction",0,1,0.03)

plane = p.loadURDF("plane_with_restitution.urdf")
sphere = p.loadURDF("sphere_with_restitution.urdf",[0,0,2])

p.setRealTimeSimulation(1)
p.setGravity(0,0,-10)
while (1):
	restitution = p.readUserDebugParameter(restitutionId)
	restitutionVelocityThreshold = p.readUserDebugParameter(restitutionVelocityThresholdId)
	p.setPhysicsEngineParameter(restitutionVelocityThreshold=restitutionVelocityThreshold)

	lateralFriction = 	p.readUserDebugParameter(lateralFrictionId)
	spinningFriction = 	p.readUserDebugParameter(spinningFrictionId)
	rollingFriction = 	p.readUserDebugParameter(rollingFrictionId)
	p.changeDynamics(plane,-1,lateralFriction=1)
	p.changeDynamics(sphere,-1,lateralFriction=lateralFriction)
	p.changeDynamics(sphere,-1,spinningFriction=spinningFriction)
	p.changeDynamics(sphere,-1,rollingFriction=rollingFriction)
	
	p.changeDynamics(plane,-1,restitution=restitution)
	p.changeDynamics(sphere,-1,restitution=restitution)
	pos,orn=p.getBasePositionAndOrientation(sphere)
	#print("pos=")
	#print(pos)
	time.sleep(0.01)