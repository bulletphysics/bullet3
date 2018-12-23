
import pybullet as p
from pdControllerExplicit import PDControllerExplicit
from pdControllerExplicit import PDControllerStable

import time


useMaximalCoordinates=False
p.connect(p.GUI)
pole = p.loadURDF("cartpole.urdf", [0,0,0], useMaximalCoordinates=useMaximalCoordinates)
pole2 = p.loadURDF("cartpole.urdf", [0,1,0], useMaximalCoordinates=useMaximalCoordinates)
pole3 = p.loadURDF("cartpole.urdf", [0,2,0], useMaximalCoordinates=useMaximalCoordinates)
pole4 = p.loadURDF("cartpole.urdf", [0,3,0], useMaximalCoordinates=useMaximalCoordinates)

exPD = PDControllerExplicit(p)
sPD = PDControllerStable(p)


for i in range (p.getNumJoints(pole2)):
	#disable default constraint-based motors
	p.setJointMotorControl2(pole,i,p.POSITION_CONTROL,targetPosition=0,force=0)
	p.setJointMotorControl2(pole2,i,p.POSITION_CONTROL,targetPosition=0,force=0)
	p.setJointMotorControl2(pole3,i,p.POSITION_CONTROL,targetPosition=0,force=0)
	p.setJointMotorControl2(pole4,i,p.POSITION_CONTROL,targetPosition=0,force=0)
	
	#print("joint",i,"=",p.getJointInfo(pole2,i))


timeStepId = p.addUserDebugParameter("timeStep",0.001,0.1,0.01)
desiredPosCartId = p.addUserDebugParameter("desiredPosCart",-10,10,2)
desiredVelCartId = p.addUserDebugParameter("desiredVelCart",-10,10,0)
kpCartId = p.addUserDebugParameter("kpCart",0,500,1300)
kdCartId = p.addUserDebugParameter("kdCart",0,300,150)
maxForceCartId = p.addUserDebugParameter("maxForceCart",0,5000,1000)

textColor = [1,1,1]
shift = 0.05
p.addUserDebugText("explicit PD", [shift,0,.1],textColor,parentObjectUniqueId=pole,parentLinkIndex=1)
p.addUserDebugText("explicit PD plugin", [shift,0,-.1],textColor,parentObjectUniqueId=pole2,parentLinkIndex=1)
p.addUserDebugText("stablePD", [shift,0,.1],textColor,parentObjectUniqueId=pole4,parentLinkIndex=1)
p.addUserDebugText("position constraint", [shift,0,-.1],textColor,parentObjectUniqueId=pole3,parentLinkIndex=1)

desiredPosPoleId = p.addUserDebugParameter("desiredPosPole",-10,10,0)
desiredVelPoleId = p.addUserDebugParameter("desiredVelPole",-10,10,0)
kpPoleId = p.addUserDebugParameter("kpPole",0,500,1200)
kdPoleId = p.addUserDebugParameter("kdPole",0,300,100)
maxForcePoleId = p.addUserDebugParameter("maxForcePole",0,5000,1000)

pd = p.loadPlugin("pdControlPlugin")

	
p.setGravity(0,0,-10)

useRealTimeSim = False

p.setRealTimeSimulation(useRealTimeSim)
	
timeStep = 0.001
	
	
while p.isConnected():
	p.getCameraImage(320,200)
	timeStep = p.readUserDebugParameter(timeStepId)
	p.setTimeStep(timeStep)

	desiredPosCart = p.readUserDebugParameter(desiredPosCartId)
	desiredVelCart = p.readUserDebugParameter(desiredVelCartId)
	kpCart = p.readUserDebugParameter(kpCartId)
	kdCart = p.readUserDebugParameter(kdCartId)
	maxForceCart = p.readUserDebugParameter(maxForceCartId)

	desiredPosPole = p.readUserDebugParameter(desiredPosPoleId)
	desiredVelPole = p.readUserDebugParameter(desiredVelPoleId)
	kpPole = p.readUserDebugParameter(kpPoleId)
	kdPole = p.readUserDebugParameter(kdPoleId)
	maxForcePole = p.readUserDebugParameter(maxForcePoleId)
	
	taus = exPD.computePD(pole, [0,1], [desiredPosCart,desiredPosPole],[desiredVelCart,desiredVelPole], [kpCart,kpPole], [kdCart,kdPole],[maxForceCart,maxForcePole], timeStep)
	p.setJointMotorControlArray(pole, [0,1], controlMode=p.TORQUE_CONTROL, forces=taus)
		
	if (pd>=0):
		link = 0
		p.setJointMotorControl2(bodyUniqueId=pole2,jointIndex=link,controlMode=p.PD_CONTROL,targetPosition=desiredPosCart,targetVelocity=desiredVelCart,force=maxForceCart, positionGain=kpCart, velocityGain=kdCart)
		link = 1
		p.setJointMotorControl2(bodyUniqueId=pole2,jointIndex=link,controlMode=p.PD_CONTROL,targetPosition=desiredPosPole,targetVelocity=desiredVelPole,force=maxForcePole, positionGain=kpPole, velocityGain=kdPole)
	
	
	
	
	taus = sPD.computePD(pole4, [0,1], [desiredPosCart,desiredPosPole],[desiredVelCart,desiredVelPole], [kpCart,kpPole], [kdCart,kdPole],[maxForceCart,maxForcePole], timeStep)
	p.setJointMotorControlArray(pole4, [0,1], controlMode=p.TORQUE_CONTROL, forces=taus)
		
	p.setJointMotorControl2(pole3,0, p.POSITION_CONTROL, targetPosition=desiredPosCart, targetVelocity=desiredVelCart, positionGain=timeStep*(kpCart/150.), velocityGain=0.5, force=maxForceCart)
	p.setJointMotorControl2(pole3,1, p.POSITION_CONTROL, targetPosition=desiredPosPole, targetVelocity=desiredVelPole, positionGain=timeStep*(kpPole/150.), velocityGain=0.5, force=maxForcePole)
	
	if (not useRealTimeSim):
		p.stepSimulation()
		time.sleep(timeStep)
	
	
