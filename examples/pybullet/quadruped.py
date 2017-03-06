import pybullet as p
import time
import math

useRealTime = 0
fixedTimeStep = 0.001
speed = 10
amplitude = 0.8
jump_amp = 0.5
maxForce = 3.5
kneeFrictionForce = 0.00
kp = .05
kd = .5


physId = p.connect(p.SHARED_MEMORY)
if (physId<0):
	p.connect(p.GUI)
#p.resetSimulation()
p.loadURDF("plane.urdf",0,0,0.1)
#p.loadSDF("kitchens/1.sdf")
p.setGravity(0,0,0)
p.setTimeStep(fixedTimeStep)

p.setRealTimeSimulation(0)
quadruped = p.loadURDF("quadruped/minitaur.urdf",[0,0,0.2],useFixedBase=False)
nJoints = p.getNumJoints(quadruped)

jointNameToId = {}
for i in range(nJoints):
	jointInfo = p.getJointInfo(quadruped, i)
	jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]


motor_front_rightR_joint = jointNameToId['motor_front_rightR_joint']
hip_front_rightR_link = jointNameToId['hip_front_rightR_link']
knee_front_rightR_link = jointNameToId['knee_front_rightR_link']
motor_front_rightL_joint = jointNameToId['motor_front_rightL_joint']
motor_front_rightL_link = jointNameToId['motor_front_rightL_link']
knee_front_rightL_link = jointNameToId['knee_front_rightL_link']
motor_front_leftR_joint = jointNameToId['motor_front_leftR_joint']
hip_front_leftR_link = jointNameToId['hip_front_leftR_link']
knee_front_leftR_link = jointNameToId['knee_front_leftR_link']
motor_front_leftL_joint = jointNameToId['motor_front_leftL_joint']
motor_front_leftL_link = jointNameToId['motor_front_leftL_link']
knee_front_leftL_link = jointNameToId['knee_front_leftL_link']
motor_back_rightR_joint = jointNameToId['motor_back_rightR_joint']
hip_rightR_link = jointNameToId['hip_rightR_link']
knee_back_rightR_link = jointNameToId['knee_back_rightR_link']
motor_back_rightL_joint = jointNameToId['motor_back_rightL_joint']
motor_back_rightL_link = jointNameToId['motor_back_rightL_link']
knee_back_rightL_link = jointNameToId['knee_back_rightL_link']
motor_back_leftR_joint = jointNameToId['motor_back_leftR_joint']
hip_leftR_link = jointNameToId['hip_leftR_link']
knee_back_leftR_link = jointNameToId['knee_back_leftR_link']
motor_back_leftL_joint = jointNameToId['motor_back_leftL_joint']
motor_back_leftL_link = jointNameToId['motor_back_leftL_link']
knee_back_leftL_link = jointNameToId['knee_back_leftL_link']


#p.getNumJoints(1)
p.resetJointState(quadruped,motor_front_rightR_joint,1.57)
p.resetJointState(quadruped,knee_front_rightR_link,-2.2)
p.resetJointState(quadruped,motor_front_rightL_joint,1.57)
p.resetJointState(quadruped,knee_front_rightL_link,-2.2)
p.createConstraint(quadruped,knee_front_rightR_link,quadruped,knee_front_rightL_link,p.JOINT_POINT2POINT,[0,0,0],[0,0.005,0.2],[0,0.01,0.2])
p.setJointMotorControl(quadruped,knee_front_rightR_link,p.VELOCITY_CONTROL,0,kneeFrictionForce)
p.setJointMotorControl(quadruped,knee_front_rightL_link,p.VELOCITY_CONTROL,0,kneeFrictionForce)

p.resetJointState(quadruped,motor_front_leftR_joint,-1.57)
p.resetJointState(quadruped,knee_front_leftR_link,2.2)
p.resetJointState(quadruped,motor_front_leftL_joint,-1.57)
p.resetJointState(quadruped,knee_front_leftL_link,2.2)
p.createConstraint(quadruped,knee_front_leftR_link,quadruped,knee_front_leftL_link,p.JOINT_POINT2POINT,[0,0,0],[0,0.005,0.2],[0,0.01,0.2])
p.setJointMotorControl(quadruped,knee_front_leftR_link,p.VELOCITY_CONTROL,0,kneeFrictionForce)
p.setJointMotorControl(quadruped,knee_front_leftL_link,p.VELOCITY_CONTROL,0,kneeFrictionForce)

p.resetJointState(quadruped,motor_back_rightR_joint,1.57)
p.resetJointState(quadruped,knee_back_rightR_link,-2.2)
p.resetJointState(quadruped,motor_back_rightL_joint,1.57)
p.resetJointState(quadruped,knee_back_rightL_link,-2.2)
p.createConstraint(quadruped,knee_back_rightR_link,quadruped,knee_back_rightL_link,p.JOINT_POINT2POINT,[0,0,0],[0,0.005,0.2],[0,0.01,0.2])
p.setJointMotorControl(quadruped,knee_back_rightR_link,p.VELOCITY_CONTROL,0,kneeFrictionForce)
p.setJointMotorControl(quadruped,knee_back_rightL_link,p.VELOCITY_CONTROL,0,kneeFrictionForce)

p.resetJointState(quadruped,motor_back_leftR_joint,-1.57)
p.resetJointState(quadruped,knee_back_leftR_link,2.2)
p.resetJointState(quadruped,motor_back_leftL_joint,-1.57)
p.resetJointState(quadruped,knee_back_leftL_link,2.2)
p.createConstraint(quadruped,knee_back_leftR_link,quadruped,knee_back_leftL_link,p.JOINT_POINT2POINT,[0,0,0],[0,0.005,0.2],[0,0.01,0.2])
p.setJointMotorControl(quadruped,motor_back_leftR_joint,p.POSITION_CONTROL,-1.57,maxForce)
p.setJointMotorControl(quadruped,knee_back_leftR_link,p.VELOCITY_CONTROL,0,kneeFrictionForce)
p.setJointMotorControl(quadruped,motor_back_leftL_joint,p.POSITION_CONTROL,-1.57,maxForce)
p.setJointMotorControl(quadruped,knee_back_leftL_link,p.VELOCITY_CONTROL,0,kneeFrictionForce)



motordir=[-1,-1,-1,-1,1,1,1,1]
legnumbering=[motor_front_leftR_joint,motor_front_leftL_joint,motor_back_leftR_joint,motor_back_leftL_joint,motor_front_rightR_joint,motor_front_rightL_joint,motor_back_rightR_joint,motor_back_rightL_joint]

#use the Minitaur leg numbering
p.setJointMotorControl2(bodyIndex=quadruped,jointIndex=legnumbering[0],controlMode=p.POSITION_CONTROL,targetPosition=motordir[0]*1.57,positionGain=kp, velocityGain=kd, force=maxForce)
p.setJointMotorControl2(bodyIndex=quadruped,jointIndex=legnumbering[1],controlMode=p.POSITION_CONTROL,targetPosition=motordir[1]*1.57,positionGain=kp, velocityGain=kd, force=maxForce)
p.setJointMotorControl2(bodyIndex=quadruped,jointIndex=legnumbering[2],controlMode=p.POSITION_CONTROL,targetPosition=motordir[2]*1.57,positionGain=kp, velocityGain=kd, force=maxForce)
p.setJointMotorControl2(bodyIndex=quadruped,jointIndex=legnumbering[3],controlMode=p.POSITION_CONTROL,targetPosition=motordir[3]*1.57,positionGain=kp, velocityGain=kd, force=maxForce)
p.setJointMotorControl2(bodyIndex=quadruped,jointIndex=legnumbering[4],controlMode=p.POSITION_CONTROL,targetPosition=motordir[4]*1.57,positionGain=kp, velocityGain=kd, force=maxForce)
p.setJointMotorControl2(bodyIndex=quadruped,jointIndex=legnumbering[5],controlMode=p.POSITION_CONTROL,targetPosition=motordir[5]*1.57,positionGain=kp, velocityGain=kd, force=maxForce)
p.setJointMotorControl2(bodyIndex=quadruped,jointIndex=legnumbering[6],controlMode=p.POSITION_CONTROL,targetPosition=motordir[6]*1.57,positionGain=kp, velocityGain=kd, force=maxForce)
p.setJointMotorControl2(bodyIndex=quadruped,jointIndex=legnumbering[7],controlMode=p.POSITION_CONTROL,targetPosition=motordir[7]*1.57,positionGain=kp, velocityGain=kd, force=maxForce)


p.stepSimulation()


print("quadruped Id = ")
print(quadruped)
p.saveWorld("quadru.py")
logId = p.startStateLogging(p.STATE_LOGGING_MINITAUR,"quadrupedLog.txt",[quadruped])

p.setGravity(0,0,-10)


#stand still
p.setRealTimeSimulation(1)

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

	p.setJointMotorControl2(bodyIndex=quadruped, jointIndex=motor_front_rightR_joint, controlMode=p.POSITION_CONTROL, targetPosition=math.sin(t*speed)*jump_amp+1.57, positionGain=kp, velocityGain=kd, force=maxForce)
	p.setJointMotorControl2(bodyIndex=quadruped, jointIndex=motor_front_rightL_joint, controlMode=p.POSITION_CONTROL, targetPosition=-math.sin(t*speed)*jump_amp-1.57, positionGain=kp, velocityGain=kd, force=maxForce)
	p.setJointMotorControl2(bodyIndex=quadruped, jointIndex=motor_front_leftR_joint,  controlMode=p.POSITION_CONTROL, targetPosition=math.sin(t*speed)*jump_amp+1.57, positionGain=kp, velocityGain=kd, force=maxForce)
	p.setJointMotorControl2(bodyIndex=quadruped, jointIndex=motor_front_leftL_joint,  controlMode=p.POSITION_CONTROL, targetPosition=-math.sin(t*speed)*jump_amp-1.57, positionGain=kp, velocityGain=kd, force=maxForce)
	p.setJointMotorControl2(bodyIndex=quadruped, jointIndex=motor_back_rightR_joint,  controlMode=p.POSITION_CONTROL, targetPosition=math.sin(t*speed)*jump_amp+1.57, positionGain=kp, velocityGain=kd, force=maxForce)
	p.setJointMotorControl2(bodyIndex=quadruped, jointIndex=motor_back_rightL_joint,  controlMode=p.POSITION_CONTROL, targetPosition=-math.sin(t*speed)*jump_amp-1.57, positionGain=kp, velocityGain=kd, force=maxForce)
	p.setJointMotorControl2(bodyIndex=quadruped, jointIndex=motor_back_leftR_joint,   controlMode=p.POSITION_CONTROL, targetPosition=math.sin(t*speed)*jump_amp+1.57, positionGain=kp, velocityGain=kd, force=maxForce)
	p.setJointMotorControl2(bodyIndex=quadruped, jointIndex=motor_back_leftL_joint,   controlMode=p.POSITION_CONTROL, targetPosition=-math.sin(t*speed)*jump_amp-1.57, positionGain=kp, velocityGain=kd, force=maxForce)

	target = math.sin(t*speed)*jump_amp+1.57;
	p.setJointMotorControl2(bodyIndex=quadruped,jointIndex=legnumbering[0],controlMode=p.POSITION_CONTROL,targetPosition=motordir[0]*target,positionGain=kp, velocityGain=kd, force=maxForce)
	p.setJointMotorControl2(bodyIndex=quadruped,jointIndex=legnumbering[1],controlMode=p.POSITION_CONTROL,targetPosition=motordir[1]*target,positionGain=kp, velocityGain=kd, force=maxForce)
	p.setJointMotorControl2(bodyIndex=quadruped,jointIndex=legnumbering[2],controlMode=p.POSITION_CONTROL,targetPosition=motordir[2]*target,positionGain=kp, velocityGain=kd, force=maxForce)
	p.setJointMotorControl2(bodyIndex=quadruped,jointIndex=legnumbering[3],controlMode=p.POSITION_CONTROL,targetPosition=motordir[3]*target,positionGain=kp, velocityGain=kd, force=maxForce)
	p.setJointMotorControl2(bodyIndex=quadruped,jointIndex=legnumbering[4],controlMode=p.POSITION_CONTROL,targetPosition=motordir[4]*target,positionGain=kp, velocityGain=kd, force=maxForce)
	p.setJointMotorControl2(bodyIndex=quadruped,jointIndex=legnumbering[5],controlMode=p.POSITION_CONTROL,targetPosition=motordir[5]*target,positionGain=kp, velocityGain=kd, force=maxForce)
	p.setJointMotorControl2(bodyIndex=quadruped,jointIndex=legnumbering[6],controlMode=p.POSITION_CONTROL,targetPosition=motordir[6]*target,positionGain=kp, velocityGain=kd, force=maxForce)
	p.setJointMotorControl2(bodyIndex=quadruped,jointIndex=legnumbering[7],controlMode=p.POSITION_CONTROL,targetPosition=motordir[7]*target,positionGain=kp, velocityGain=kd, force=maxForce)
	
	if (useRealTime==0):	
		p.stepSimulation()

