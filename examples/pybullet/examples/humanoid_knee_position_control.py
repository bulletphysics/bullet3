import pybullet as p
import time

cid = p.connect(p.SHARED_MEMORY)
if (cid<0):
	cid = p.connect(p.GUI)

p.resetSimulation()

useRealTime = 0

p.setRealTimeSimulation(useRealTime)

p.setGravity(0,0,0)

p.loadURDF("plane.urdf")

obUids = p.loadMJCF("mjcf/humanoid_fixed.xml")
human = obUids[0]



for i in range (p.getNumJoints(human)):
	p.setJointMotorControl2(human,i,p.POSITION_CONTROL,targetPosition=0,force=50)

kneeAngleTargetId = p.addUserDebugParameter("kneeAngle",-4,4,-1)
maxForceId = p.addUserDebugParameter("maxForce",0,100,10)

kneeJointIndex=11

while (1):
	time.sleep(0.01)
	kneeAngleTarget = p.readUserDebugParameter(kneeAngleTargetId)
	maxForce = p.readUserDebugParameter(maxForceId)
	p.setJointMotorControl2(human,kneeJointIndex,p.POSITION_CONTROL,targetPosition=kneeAngleTarget,force=maxForce)
	if (useRealTime==0):
		p.stepSimulation()	