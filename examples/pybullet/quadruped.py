import pybullet as p
import time
import math

useRealTime = 0
fixedTimeStep = 0.001

physId = p.connect(p.SHARED_MEMORY)
if (physId<0):
	p.connect(p.GUI)

p.loadURDF("plane.urdf")
p.setGravity(0,0,-1)
p.setTimeStep(fixedTimeStep)

p.setRealTimeSimulation(0)
quadruped = p.loadURDF("quadruped/quadruped.urdf",1,-2,1)
#p.getNumJoints(1)
#right front leg
p.resetJointState(quadruped,0,1.57)
p.resetJointState(quadruped,2,-2.2)
p.resetJointState(quadruped,3,-1.57)
p.resetJointState(quadruped,5,2.2)
p.createConstraint(quadruped,2,quadruped,5,p.JOINT_POINT2POINT,[0,0,0],[0,0.01,0.2],[0,-0.015,0.2])

p.setJointMotorControl(quadruped,0,p.POSITION_CONTROL,1.57,1)
p.setJointMotorControl(quadruped,1,p.VELOCITY_CONTROL,0,0)
p.setJointMotorControl(quadruped,2,p.VELOCITY_CONTROL,0,0)
p.setJointMotorControl(quadruped,3,p.POSITION_CONTROL,-1.57,1)
p.setJointMotorControl(quadruped,4,p.VELOCITY_CONTROL,0,0)
p.setJointMotorControl(quadruped,5,p.VELOCITY_CONTROL,0,0)

#left front leg
p.resetJointState(quadruped,6,1.57)
p.resetJointState(quadruped,8,-2.2)
p.resetJointState(quadruped,9,-1.57)
p.resetJointState(quadruped,11,2.2)
p.createConstraint(quadruped,8,quadruped,11,p.JOINT_POINT2POINT,[0,0,0],[0,-0.01,0.2],[0,0.015,0.2])

p.setJointMotorControl(quadruped,6,p.POSITION_CONTROL,1.57,1)
p.setJointMotorControl(quadruped,7,p.VELOCITY_CONTROL,0,0)
p.setJointMotorControl(quadruped,8,p.VELOCITY_CONTROL,0,0)
p.setJointMotorControl(quadruped,9,p.POSITION_CONTROL,-1.57,1)
p.setJointMotorControl(quadruped,10,p.VELOCITY_CONTROL,0,0)
p.setJointMotorControl(quadruped,11,p.VELOCITY_CONTROL,0,0)


#right back leg
p.resetJointState(quadruped,12,1.57)
p.resetJointState(quadruped,14,-2.2)
p.resetJointState(quadruped,15,-1.57)
p.resetJointState(quadruped,17,2.2)
p.createConstraint(quadruped,14,quadruped,17,p.JOINT_POINT2POINT,[0,0,0],[0,0.01,0.2],[0,-0.015,0.2])

p.setJointMotorControl(quadruped,12,p.POSITION_CONTROL,1.57,1)
p.setJointMotorControl(quadruped,13,p.VELOCITY_CONTROL,0,0)
p.setJointMotorControl(quadruped,14,p.VELOCITY_CONTROL,0,0)
p.setJointMotorControl(quadruped,15,p.POSITION_CONTROL,-1.57,1)
p.setJointMotorControl(quadruped,16,p.VELOCITY_CONTROL,0,0)
p.setJointMotorControl(quadruped,17,p.VELOCITY_CONTROL,0,0)

#left back leg
p.resetJointState(quadruped,18,1.57)
p.resetJointState(quadruped,20,-2.2)
p.resetJointState(quadruped,21,-1.57)
p.resetJointState(quadruped,23,2.2)
p.createConstraint(quadruped,20,quadruped,23,p.JOINT_POINT2POINT,[0,0,0],[0,-0.01,0.2],[0,0.015,0.2])

p.setJointMotorControl(quadruped,18,p.POSITION_CONTROL,1.57,1)
p.setJointMotorControl(quadruped,19,p.VELOCITY_CONTROL,0,0)
p.setJointMotorControl(quadruped,20,p.VELOCITY_CONTROL,0,0)
p.setJointMotorControl(quadruped,21,p.POSITION_CONTROL,-1.57,1)
p.setJointMotorControl(quadruped,22,p.VELOCITY_CONTROL,0,0)
p.setJointMotorControl(quadruped,23,p.VELOCITY_CONTROL,0,0)



p_gain =1 
speed = 10 
amplitude = 1.3

#stand still
p.setRealTimeSimulation(useRealTime)


t=0.0
ref_time = time.time()
t_end = t + 4
while t < t_end:
	if (useRealTime==0):
		t = t+fixedTimeStep
		p.stepSimulation()
	else:
		t = time.time()-ref_time
		p.setGravity(0,0,-1)

p.setGravity(0,0,-10)

jump_amp = 0.5

#jump
t = 0.0
t_end = t + 10
i=0
ref_time = time.time()

while t < t_end:
	if (useRealTime):
		t = time.time()-ref_time
	else:
		t = t+fixedTimeStep

	p.setJointMotorControl(quadruped,0,p.POSITION_CONTROL,math.sin(t*speed)*jump_amp+1.57,p_gain)
	p.setJointMotorControl(quadruped,3,p.POSITION_CONTROL,-math.sin(t*speed)*jump_amp-1.57,p_gain)
	p.setJointMotorControl(quadruped,6,p.POSITION_CONTROL,math.sin(t*speed)*jump_amp+1.57,p_gain)
	p.setJointMotorControl(quadruped,9,p.POSITION_CONTROL,-math.sin(t*speed)*jump_amp-1.57,p_gain)
	p.setJointMotorControl(quadruped,12,p.POSITION_CONTROL,math.sin(t*speed)*jump_amp+1.57,p_gain)
	p.setJointMotorControl(quadruped,15,p.POSITION_CONTROL,-math.sin(t*speed)*jump_amp-1.57,p_gain)
	p.setJointMotorControl(quadruped,18,p.POSITION_CONTROL,math.sin(t*speed)*jump_amp+1.57,p_gain)
	p.setJointMotorControl(quadruped,21,p.POSITION_CONTROL,-math.sin(t*speed)*jump_amp-1.57,p_gain)
	if (useRealTime==0):	
		p.stepSimulation()


#hop forward
t_end = 20
i=0
while t < t_end:
	if (useRealTime):
		t = time.time()-ref_time
	else:
		t = t+fixedTimeStep

	p.setJointMotorControl(quadruped,0,p.POSITION_CONTROL,math.sin(t*speed)*amplitude+1.57,p_gain)
	p.setJointMotorControl(quadruped,3,p.POSITION_CONTROL,-1.57,p_gain)
	p.setJointMotorControl(quadruped,6,p.POSITION_CONTROL,math.sin(t*speed)*amplitude+1.57,p_gain)
	p.setJointMotorControl(quadruped,9,p.POSITION_CONTROL,-1.57,p_gain)
	p.setJointMotorControl(quadruped,12,p.POSITION_CONTROL,math.sin(t*speed+3.14)*amplitude+1.57,p_gain)
	p.setJointMotorControl(quadruped,15,p.POSITION_CONTROL,-1.57,p_gain)
	p.setJointMotorControl(quadruped,18,p.POSITION_CONTROL,math.sin(t*speed+3.14)*amplitude+1.57,p_gain)
	p.setJointMotorControl(quadruped,21,p.POSITION_CONTROL,-1.57,p_gain)
	if (useRealTime==0):	
		p.stepSimulation()

#walk
t_end = 100
i=0
while t < t_end:
	if (useRealTime):
		t = time.time()-ref_time
	else:
		t = t+fixedTimeStep

	p.setJointMotorControl(quadruped,0,p.POSITION_CONTROL,math.sin(t*3)*.3+1.57,1)
	p.setJointMotorControl(quadruped,3,p.POSITION_CONTROL,-1.57,1)
	p.setJointMotorControl(quadruped,6,p.POSITION_CONTROL,math.sin(t*3+0.5*3.14)*.3+1.57,1)
	p.setJointMotorControl(quadruped,9,p.POSITION_CONTROL,-1.57,1)
	p.setJointMotorControl(quadruped,12,p.POSITION_CONTROL,math.sin(t*3+3.14)*.3+1.57,1)
	p.setJointMotorControl(quadruped,15,p.POSITION_CONTROL,-1.57,1)
	p.setJointMotorControl(quadruped,18,p.POSITION_CONTROL,math.sin(t*3+1.5*3.14)*.3+1.57,1)
	p.setJointMotorControl(quadruped,21,p.POSITION_CONTROL,-1.57,1)

	p.stepSimulation()
p.setRealTimeSimulation(1)
