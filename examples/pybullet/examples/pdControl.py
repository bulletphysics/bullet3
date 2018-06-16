
import pybullet as p
import time

useMaximalCoordinates=False
p.connect(p.GUI)
pole = p.loadURDF("cartpole.urdf", useMaximalCoordinates=useMaximalCoordinates)
for i in range (p.getNumJoints(pole)):
	#disable default constraint-based motors
	p.setJointMotorControl2(pole,i,p.POSITION_CONTROL,targetPosition=0,force=0)
	print("joint",i,"=",p.getJointInfo(pole,i))


desiredPosCartId = p.addUserDebugParameter("desiredPosCart",-10,10,2)
desiredVelCartId = p.addUserDebugParameter("desiredVelCart",-10,10,0)
kpCartId = p.addUserDebugParameter("kpCart",0,500,300)
kdCartId = p.addUserDebugParameter("kdCart",0,300,150)
maxForceCartId = p.addUserDebugParameter("maxForceCart",0,5000,1000)


desiredPosPoleId = p.addUserDebugParameter("desiredPosPole",-10,10,0)
desiredVelPoleId = p.addUserDebugParameter("desiredVelPole",-10,10,0)
kpPoleId = p.addUserDebugParameter("kpPole",0,500,200)
kdPoleId = p.addUserDebugParameter("kdPole",0,300,100)
maxForcePoleId = p.addUserDebugParameter("maxForcePole",0,5000,1000)

pd = p.loadPlugin("pdControlPlugin")

	
p.setGravity(0,0,-10)

useRealTimeSim = True

p.setRealTimeSimulation(useRealTimeSim)
	
p.setTimeStep(0.001)
	
	
while p.isConnected():
	if (pd>=0):
		desiredPosCart = p.readUserDebugParameter(desiredPosCartId)
		desiredVelCart = p.readUserDebugParameter(desiredVelCartId)
		kpCart = p.readUserDebugParameter(kpCartId)
		kdCart = p.readUserDebugParameter(kdCartId)
		maxForceCart = p.readUserDebugParameter(maxForceCartId)
		link = 0
		p.setJointMotorControl2(bodyUniqueId=pole,jointIndex=link,controlMode=p.PD_CONTROL,targetPosition=desiredPosCart,targetVelocity=desiredVelCart,force=maxForceCart, positionGain=kpCart, velocityGain=kdCart)
		
		desiredPosPole = p.readUserDebugParameter(desiredPosPoleId)
		desiredVelPole = p.readUserDebugParameter(desiredVelPoleId)
		kpPole = p.readUserDebugParameter(kpPoleId)
		kdPole = p.readUserDebugParameter(kdPoleId)
		maxForcePole = p.readUserDebugParameter(maxForcePoleId)
		link = 1
		p.setJointMotorControl2(bodyUniqueId=pole,jointIndex=link,controlMode=p.PD_CONTROL,targetPosition=desiredPosPole,targetVelocity=desiredVelPole,force=maxForcePole, positionGain=kpPole, velocityGain=kdPole)
		

	if (not useRealTimeSim):
		p.stepSimulation()
		time.sleep(1./240.)
	
	
