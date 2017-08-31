import pybullet as p
import time
import math

useRealTime = 0
fixedTimeStep = 0.01
speed = 10
amplitude = 0.8
jump_amp = 0.5
maxForce = 3.5
kneeFrictionForce = 0.00
kp = 1
kd = .1


physId = p.connect(p.SHARED_MEMORY)
if (physId<0):
	p.connect(p.GUI)
#p.resetSimulation()
p.loadURDF("plane.urdf",0,0,0)
p.setPhysicsEngineParameter(numSolverIterations=50)
p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT, "genericlogdata.bin", maxLogDof = 16, logFlags = p.STATE_LOG_JOINT_TORQUES)
p.setTimeOut(4)

p.setGravity(0,0,0)
p.setTimeStep(fixedTimeStep)

orn = p.getQuaternionFromEuler([0,0,0.4])
p.setRealTimeSimulation(0)
quadruped = p.loadURDF("quadruped/minitaur.urdf",[1,0,0.2],orn,useFixedBase=False)
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

motordir=[-1,-1,-1,-1,1,1,1,1]
halfpi = 1.57079632679
kneeangle = -2.1834
p.resetJointState(quadruped,motor_front_leftL_joint,motordir[0]*halfpi)
p.resetJointState(quadruped,knee_front_leftL_link,motordir[0]*kneeangle)
p.resetJointState(quadruped,motor_front_leftR_joint,motordir[1]*halfpi)
p.resetJointState(quadruped,knee_front_leftR_link,motordir[1]*kneeangle)
cid = p.createConstraint(quadruped,knee_front_leftR_link,quadruped,knee_front_leftL_link,p.JOINT_POINT2POINT,[0,0,0],[0,0.005,0.2],[0,0.01,0.2])
p.changeConstraint(cid,maxForce=10000)

p.setJointMotorControl(quadruped,knee_front_leftL_link,p.VELOCITY_CONTROL,0,kneeFrictionForce)
p.setJointMotorControl(quadruped,knee_front_leftR_link,p.VELOCITY_CONTROL,0,kneeFrictionForce)

p.resetJointState(quadruped,motor_back_leftL_joint,motordir[2]*halfpi)
p.resetJointState(quadruped,knee_back_leftL_link,motordir[2]*kneeangle)
p.resetJointState(quadruped,motor_back_leftR_joint,motordir[3]*halfpi)
p.resetJointState(quadruped,knee_back_leftR_link,motordir[3]*kneeangle)
cid = p.createConstraint(quadruped,knee_back_leftR_link,quadruped,knee_back_leftL_link,p.JOINT_POINT2POINT,[0,0,0],[0,0.005,0.2],[0,0.01,0.2])
p.changeConstraint(cid,maxForce=10000)
p.setJointMotorControl(quadruped,knee_back_leftL_link,p.VELOCITY_CONTROL,0,kneeFrictionForce)
p.setJointMotorControl(quadruped,knee_back_leftR_link,p.VELOCITY_CONTROL,0,kneeFrictionForce)


#p.getNumJoints(1)


p.resetJointState(quadruped,motor_front_rightL_joint,motordir[4]*halfpi)
p.resetJointState(quadruped,knee_front_rightL_link,motordir[4]*kneeangle)
p.resetJointState(quadruped,motor_front_rightR_joint,motordir[5]*halfpi)
p.resetJointState(quadruped,knee_front_rightR_link,motordir[5]*kneeangle)
cid = p.createConstraint(quadruped,knee_front_rightR_link,quadruped,knee_front_rightL_link,p.JOINT_POINT2POINT,[0,0,0],[0,0.005,0.2],[0,0.01,0.2])
p.changeConstraint(cid,maxForce=10000)

p.setJointMotorControl(quadruped,knee_front_rightL_link,p.VELOCITY_CONTROL,0,kneeFrictionForce)
p.setJointMotorControl(quadruped,knee_front_rightR_link,p.VELOCITY_CONTROL,0,kneeFrictionForce)

p.resetJointState(quadruped,motor_back_rightL_joint,motordir[6]*halfpi)
p.resetJointState(quadruped,knee_back_rightL_link,motordir[6]*kneeangle)
p.resetJointState(quadruped,motor_back_rightR_joint,motordir[7]*halfpi)
p.resetJointState(quadruped,knee_back_rightR_link,motordir[7]*kneeangle)
cid = p.createConstraint(quadruped,knee_back_rightR_link,quadruped,knee_back_rightL_link,p.JOINT_POINT2POINT,[0,0,0],[0,0.005,0.2],[0,0.01,0.2])
p.changeConstraint(cid,maxForce=10000)

p.setJointMotorControl(quadruped,knee_back_rightL_link,p.VELOCITY_CONTROL,0,kneeFrictionForce)
p.setJointMotorControl(quadruped,knee_back_rightR_link,p.VELOCITY_CONTROL,0,kneeFrictionForce)

p.setGravity(0,0,-10)



legnumbering=[
	motor_front_leftL_joint,
	motor_front_leftR_joint,
	motor_back_leftL_joint,
	motor_back_leftR_joint,
	motor_front_rightL_joint,
	motor_front_rightR_joint,
	motor_back_rightL_joint,
	motor_back_rightR_joint]

for i in range (8):
	print (legnumbering[i])
#use the Minitaur leg numbering
p.setJointMotorControl2(bodyIndex=quadruped,jointIndex=legnumbering[0],controlMode=p.POSITION_CONTROL,targetPosition=motordir[0]*1.57,positionGain=kp, velocityGain=kd, force=maxForce)
p.setJointMotorControl2(bodyIndex=quadruped,jointIndex=legnumbering[1],controlMode=p.POSITION_CONTROL,targetPosition=motordir[1]*1.57,positionGain=kp, velocityGain=kd, force=maxForce)
p.setJointMotorControl2(bodyIndex=quadruped,jointIndex=legnumbering[2],controlMode=p.POSITION_CONTROL,targetPosition=motordir[2]*1.57,positionGain=kp, velocityGain=kd, force=maxForce)
p.setJointMotorControl2(bodyIndex=quadruped,jointIndex=legnumbering[3],controlMode=p.POSITION_CONTROL,targetPosition=motordir[3]*1.57,positionGain=kp, velocityGain=kd, force=maxForce)
p.setJointMotorControl2(bodyIndex=quadruped,jointIndex=legnumbering[4],controlMode=p.POSITION_CONTROL,targetPosition=motordir[4]*1.57,positionGain=kp, velocityGain=kd, force=maxForce)
p.setJointMotorControl2(bodyIndex=quadruped,jointIndex=legnumbering[5],controlMode=p.POSITION_CONTROL,targetPosition=motordir[5]*1.57,positionGain=kp, velocityGain=kd, force=maxForce)
p.setJointMotorControl2(bodyIndex=quadruped,jointIndex=legnumbering[6],controlMode=p.POSITION_CONTROL,targetPosition=motordir[6]*1.57,positionGain=kp, velocityGain=kd, force=maxForce)
p.setJointMotorControl2(bodyIndex=quadruped,jointIndex=legnumbering[7],controlMode=p.POSITION_CONTROL,targetPosition=motordir[7]*1.57,positionGain=kp, velocityGain=kd, force=maxForce)
#stand still
p.setRealTimeSimulation(useRealTime)

#while (True):
#	time.sleep(0.01)
#p.stepSimulation()


print("quadruped Id = ")
print(quadruped)
p.saveWorld("quadru.py")
logId = p.startStateLogging(p.STATE_LOGGING_MINITAUR,"quadrupedLog.bin",[quadruped])






#jump
t = 0.0
t_end = t + 100
i=0
ref_time = time.time()

for aa in range (100):
	if (useRealTime):
		t = time.time()-ref_time
	else:
		t = t+fixedTimeStep
	if (True):
	
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
		time.sleep(fixedTimeStep)

