import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)

import pybullet as p
import math
import time
import pybullet_data

p.connect(p.GUI)
#p.loadURDF("wheel.urdf",[0,0,3])
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane=p.loadURDF("plane100.urdf",[0,0,0])
timestep = 1./240.

bike=-1
for i in range (1):

	bike=p.loadURDF("bicycle/bike.urdf",[0,0+3*i,1.5], [0,0,0,1],	useFixedBase=False)
	p.setJointMotorControl2(bike,0,p.VELOCITY_CONTROL,targetVelocity=0,force=0.05)
	#p.setJointMotorControl2(bike,1,p.VELOCITY_CONTROL,targetVelocity=5, force=1000)
	p.setJointMotorControl2(bike,1,p.VELOCITY_CONTROL,targetVelocity=5, force=0)
	p.setJointMotorControl2(bike,2,p.VELOCITY_CONTROL,targetVelocity=15, force=20)

	p.changeDynamics(plane,-1, mass=20,lateralFriction=1, linearDamping=0, angularDamping=0)
	p.changeDynamics(bike,1,lateralFriction=1,linearDamping=0, angularDamping=0)
	p.changeDynamics(bike,2,lateralFriction=1,linearDamping=0, angularDamping=0)
	#p.resetJointState(bike,1,0,100)
	#p.resetJointState(bike,2,0,100)
	#p.resetBaseVelocity(bike,[0,0,0],[0,0,0])
#p.setPhysicsEngineParameter(numSubSteps=0)
#bike=p.loadURDF("frame.urdf",useFixedBase=True)
#bike = p.loadURDF("handlebar.urdf", useFixedBase=True)
#p.loadURDF("handlebar.urdf",[0,2,1])
#coord	=	p.loadURDF("handlebar.urdf", [0.7700000000000005,	0, 0.22000000000000006],useFixedBase=True)#	p.loadURDF("coordinateframe.urdf",[-2,0,1],useFixedBase=True)
#coord	=	p.loadURDF("coordinateframe.urdf",[-2,0,1],useFixedBase=True)
p.setGravity(0,0,-10)
p.setRealTimeSimulation(0)
#coordPos	=	[0,0,0]
#coordOrnEuler = [0,0,0]

#coordPos= [0.7000000000000004, 0, 0.22000000000000006]
#coordOrnEuler= [0, -0.2617993877991496, 0]

coordPos= [0.07, 0, -0.6900000000000004]
coordOrnEuler= [0, 0, 0]

coordPos2= [0, 0, 0 ]
coordOrnEuler2= [0, 0, 0]

invPos,invOrn=p.invertTransform(coordPos,p.getQuaternionFromEuler(coordOrnEuler))
mPos,mOrn = p.multiplyTransforms(invPos,invOrn, coordPos2,p.getQuaternionFromEuler(coordOrnEuler2))
eul = p.getEulerFromQuaternion(mOrn)
print("rpy=\"",eul[0],eul[1],eul[2],"\" xyz=\"", mPos[0],mPos[1], mPos[2])


shift=0
gui = 1


frame=0
states=[]
states.append(p.saveState())
#observations=[]
#observations.append(obs)
				
running = True
reverting = False
p.getCameraImage(320,200)#,renderer=p.ER_BULLET_HARDWARE_OPENGL )
				
while	(1):

	updateCam = 0
	keys = p.getKeyboardEvents()
	for k,v in keys.items():
		if (reverting or (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_TRIGGERED))):
			reverting=True
			stateIndex = len(states)-1
			#print("prestateIndex=",stateIndex)
			time.sleep(timestep)
			updateCam=1
			if (stateIndex>0):
				stateIndex-=1
				states=states[:stateIndex+1]
				#observations=observations[:stateIndex+1]
				
				
				#print("states=",states)
			#print("stateIndex =",stateIndex )
			p.restoreState(states[stateIndex])
			#obs=observations[stateIndex]
			
		
			#obs, r, done, _ = env.step(a)
		if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_RELEASED)):
			reverting = False

		if (k == ord('1') and (v&p.KEY_WAS_TRIGGERED)):
			gui = not gui
		
		
		
		if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_RELEASED)):
			running=False
			
		if (running or (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_TRIGGERED))):
			running = True
				
	if (running):
		
		p.stepSimulation()
		
		updateCam=1
		time.sleep(timestep)
		pts = p.getContactPoints()
		#print("numPoints=",len(pts))
		#for point in pts:
		#	print("Point:bodyA=", point[1],"bodyB=",point[2],"linkA=",point[3],"linkB=",point[4],"dist=",point[8],"force=",point[9])
		
		states.append(p.saveState())
		#observations.append(obs)
		stateIndex = len(states)
		#print("stateIndex =",stateIndex )
		frame += 1
	if (updateCam):
		distance=5
		yaw = 0
		humanPos, humanOrn = p.getBasePositionAndOrientation(bike)
		humanBaseVel = p.getBaseVelocity(bike)
		#print("frame",frame, "humanPos=",humanPos, "humanVel=",humanBaseVel)
		if (gui):
			
			camInfo = p.getDebugVisualizerCamera()
			curTargetPos = camInfo[11]
			distance=camInfo[10]
			yaw = camInfo[8]
			pitch=camInfo[9]
			targetPos = [0.95*curTargetPos[0]+0.05*humanPos[0],0.95*curTargetPos[1]+0.05*humanPos[1],curTargetPos[2]]
			p.resetDebugVisualizerCamera(distance,yaw,pitch,targetPos);


