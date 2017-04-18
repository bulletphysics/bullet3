import pybullet as p
import time
import math
from datetime import datetime
from time import sleep

p.connect(p.GUI)
p.loadURDF("plane.urdf",[0,0,-0.3])
kukaId = p.loadURDF("kuka_iiwa/model.urdf",[0,0,0])
p.resetBasePositionAndOrientation(kukaId,[0,0,0],[0,0,0,1])
kukaEndEffectorIndex = 6
numJoints = p.getNumJoints(kukaId)

#Joint damping coefficents. Using large values for the joints that we don't want to move.
jd=[100.0,100.0,100.0,100.0,100.0,100.0,0.5]
#jd=[0.5,0.5,0.5,0.5,0.5,0.5,0.5]

p.setGravity(0,0,0)

while 1:
	p.stepSimulation()
	for i in range (1):
		pos = [0,0,1.26]
		orn = p.getQuaternionFromEuler([0,0,3.14])

		jointPoses = p.calculateInverseKinematics(kukaId,kukaEndEffectorIndex,pos,orn,jointDamping=jd)
	
	for i in range (numJoints):
		p.setJointMotorControl2(bodyIndex=kukaId,jointIndex=i,controlMode=p.POSITION_CONTROL,targetPosition=jointPoses[i],targetVelocity=0,force=500,positionGain=0.03,velocityGain=1)
	sleep(0.05)
