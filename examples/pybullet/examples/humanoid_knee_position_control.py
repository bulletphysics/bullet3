import pybullet as p
import time

cid = p.connect(p.SHARED_MEMORY)
if (cid<0):
	cid = p.connect(p.GUI)

p.resetSimulation()

useRealTime = 0

p.setRealTimeSimulation(useRealTime)

p.setGravity(0,0,-10)

p.loadSDF("stadium.sdf")

obUids = p.loadMJCF("mjcf/humanoid_fixed.xml")
human = obUids[0]



for i in range (p.getNumJoints(human)):
	p.setJointMotorControl2(human,i,p.POSITION_CONTROL,targetPosition=0,force=500)

kneeAngleTargetId = p.addUserDebugParameter("kneeAngle",-4,4,-1)
maxForceId = p.addUserDebugParameter("maxForce",0,500,10)

kneeAngleTargetLeftId = p.addUserDebugParameter("kneeAngleL",-4,4,-1)
maxForceLeftId = p.addUserDebugParameter("maxForceL",0,500,10)


kneeJointIndex=11
kneeJointIndexLeft=18

while (1):
	time.sleep(0.01)
	kneeAngleTarget = p.readUserDebugParameter(kneeAngleTargetId)
	maxForce = p.readUserDebugParameter(maxForceId)
	p.setJointMotorControl2(human,kneeJointIndex,p.POSITION_CONTROL,targetPosition=kneeAngleTarget,force=maxForce)
	kneeAngleTargetLeft = p.readUserDebugParameter(kneeAngleTargetLeftId)
	maxForceLeft = p.readUserDebugParameter(maxForceLeftId)
	p.setJointMotorControl2(human,kneeJointIndexLeft,p.POSITION_CONTROL,targetPosition=kneeAngleTargetLeft,force=maxForceLeft)

	if (useRealTime==0):
		p.stepSimulation()	