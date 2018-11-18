import pybullet as p
import json

p.connect(p.GUI)
#p.configureDebugVisualizer(p.COV_ENABLE_Y_AXIS_UP , 1)


import pybullet_data

useMotionCapture=True
useMotionCaptureReset=not useMotionCapture


p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setPhysicsEngineParameter(numSolverIterations=200)
#path = pybullet_data.getDataPath()+"/motions/humanoid3d_backflip.txt"
path = pybullet_data.getDataPath()+"/motions/humanoid3d_cartwheel.txt"

p.loadURDF("plane.urdf",[0,0,-0.03])
print("path = ", path)
with open(path, 'r') as f:
    motion_dict = json.load(f)
#print("motion_dict = ", motion_dict)
print("len motion=", len(motion_dict))
print(motion_dict['Loop'])
numFrames = len(motion_dict['Frames'])
print("#frames = ", numFrames)

frameId= p.addUserDebugParameter("frame",0,numFrames-1,0)

jointTypes = ["JOINT_REVOLUTE","JOINT_PRISMATIC",
							"JOINT_SPHERICAL","JOINT_PLANAR","JOINT_FIXED"]
							
humanoid = p.loadURDF("humanoid/humanoid.urdf", globalScaling=0.25)

for j in range (p.getNumJoints(humanoid)):
	ji = p.getJointInfo(humanoid,j)
	targetPosition=[0]
	if (ji[2] == p.JOINT_SPHERICAL):
		targetPosition=[0,0,0,1]
	#p.setJointMotorControlMultiDof(humanoid,j,p.POSITION_CONTROL,targetPosition, force=0)
	
	#print(ji)
	print("joint[",j,"].type=",jointTypes[ji[2]])
	print("joint[",j,"].name=",ji[1])


jointIds=[]
paramIds=[]
for j in range (p.getNumJoints(humanoid)):
	p.changeDynamics(humanoid,j,linearDamping=0, angularDamping=0)
	p.changeVisualShape(humanoid,j,rgbaColor=[1,1,1,1])
	info = p.getJointInfo(humanoid,j)
	#print(info)
	if (not useMotionCapture):
		jointName = info[1]
		jointType = info[2]
		if (jointType==p.JOINT_PRISMATIC or jointType==p.JOINT_REVOLUTE):
			jointIds.append(j)
			paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"),-4,4,0))
			print("jointName=",jointName, "at ", j)
		
p.changeVisualShape(humanoid,2,rgbaColor=[1,0,0,1])
chest=1
neck=2
rightShoulder=3
rightElbow=4
leftShoulder=6
leftElbow = 7
rightHip = 9
rightKnee=10
rightAnkle=11
leftHip = 12
leftKnee=13
leftAnkle=14

import time


once=True
p.getCameraImage(320,200)
maxForce=1000


while (p.isConnected()):
	frameReal = p.readUserDebugParameter(frameId)
	frame = int(frameReal)
	frameNext = frame+1
	if (frameNext >=  numFrames):
		frameNext = frame
	
	frameFraction = frameReal - frame
	#print("frameFraction=",frameFraction)
	#print("frame=",frame)
	#print("frameNext=", frameNext)
	
	#getQuaternionSlerp
	
	frameData = motion_dict['Frames'][frame]
	frameDataNext = motion_dict['Frames'][frameNext]
	
	#print("duration=",frameData[0])
	#print(pos=[frameData])
	
	basePos1Start = [frameData[1],frameData[2],frameData[3]]
	basePos1End = [frameDataNext[1],frameDataNext[2],frameDataNext[3]]
	basePos1 = [basePos1Start[0]+frameFraction*(basePos1End[0]-basePos1Start[0]), 
		basePos1Start[1]+frameFraction*(basePos1End[1]-basePos1Start[1]), 
		basePos1Start[2]+frameFraction*(basePos1End[2]-basePos1Start[2])]
	baseOrn1Start = [frameData[5],frameData[6], frameData[7],frameData[4]]
	baseOrn1Next = [frameDataNext[5],frameDataNext[6], frameDataNext[7],frameDataNext[4]]
	baseOrn1 = p.getQuaternionSlerp(baseOrn1Start,baseOrn1Next,frameFraction)
	#pre-rotate to make z-up
	y2zPos=[0,0,0.0]
	y2zOrn = p.getQuaternionFromEuler([1.57,0,0])
	basePos,baseOrn = p.multiplyTransforms(y2zPos, y2zOrn,basePos1,baseOrn1)

	p.resetBasePositionAndOrientation(humanoid, basePos,baseOrn)
	#	once=False
	chestRotStart = [frameData[9],frameData[10],frameData[11],frameData[8]]
	chestRotEnd = [frameDataNext[9],frameDataNext[10],frameDataNext[11],frameDataNext[8]]
	chestRot = p.getQuaternionSlerp(chestRotStart,chestRotEnd,frameFraction)
	
	neckRotStart = [frameData[13],frameData[14],frameData[15],frameData[12]]
	neckRotEnd= [frameDataNext[13],frameDataNext[14],frameDataNext[15],frameDataNext[12]]
	neckRot =  p.getQuaternionSlerp(neckRotStart,neckRotEnd,frameFraction)
	
	rightHipRotStart = [frameData[17],frameData[18],frameData[19],frameData[16]]
	rightHipRotEnd = [frameDataNext[17],frameDataNext[18],frameDataNext[19],frameDataNext[16]]
	rightHipRot = p.getQuaternionSlerp(rightHipRotStart,rightHipRotEnd,frameFraction)
	
	rightKneeRotStart = [frameData[20]]
	rightKneeRotEnd = [frameDataNext[20]]
	rightKneeRot = [rightKneeRotStart[0]+frameFraction*(rightKneeRotEnd[0]-rightKneeRotStart[0])]
	
	rightAnkleRotStart = [frameData[22],frameData[23],frameData[24],frameData[21]]
	rightAnkleRotEnd = [frameDataNext[22],frameDataNext[23],frameDataNext[24],frameDataNext[21]]
	rightAnkleRot =  p.getQuaternionSlerp(rightAnkleRotStart,rightAnkleRotEnd,frameFraction)
		
	rightShoulderRotStart = [frameData[26],frameData[27],frameData[28],frameData[25]]
	rightShoulderRotEnd = [frameDataNext[26],frameDataNext[27],frameDataNext[28],frameDataNext[25]]
	rightShoulderRot = p.getQuaternionSlerp(rightShoulderRotStart,rightShoulderRotEnd,frameFraction)
	
	rightElbowRotStart = [frameData[29]]
	rightElbowRotEnd = [frameDataNext[29]]
	rightElbowRot = [rightElbowRotStart[0]+frameFraction*(rightElbowRotEnd[0]-rightElbowRotStart[0])]
	
	leftHipRotStart = [frameData[31],frameData[32],frameData[33],frameData[30]]
	leftHipRotEnd = [frameDataNext[31],frameDataNext[32],frameDataNext[33],frameDataNext[30]]
	leftHipRot = p.getQuaternionSlerp(leftHipRotStart,leftHipRotEnd,frameFraction)
	
	leftKneeRotStart = [frameData[34]]
	leftKneeRotEnd = [frameDataNext[34]]
	leftKneeRot = [leftKneeRotStart[0] +frameFraction*(leftKneeRotEnd[0]-leftKneeRotStart[0]) ]
	
	leftAnkleRotStart = [frameData[36],frameData[37],frameData[38],frameData[35]]
	leftAnkleRotEnd = [frameDataNext[36],frameDataNext[37],frameDataNext[38],frameDataNext[35]]
	leftAnkleRot = p.getQuaternionSlerp(leftAnkleRotStart,leftAnkleRotEnd,frameFraction)
	
	leftShoulderRotStart = [frameData[40],frameData[41],frameData[42],frameData[39]]
	leftShoulderRotEnd = [frameDataNext[40],frameDataNext[41],frameDataNext[42],frameDataNext[39]]
	leftShoulderRot = p.getQuaternionSlerp(leftShoulderRotStart,leftShoulderRotEnd,frameFraction)
	leftElbowRotStart = [frameData[43]]
	leftElbowRotEnd = [frameDataNext[43]]
	leftElbowRot = [leftElbowRotStart[0]+frameFraction*(leftElbowRotEnd[0]-leftElbowRotStart[0])]
	
	#print("chestRot=",chestRot)
	p.setGravity(0,0,0)
	
	
	kp=1
	if (useMotionCapture):
		
		p.setJointMotorControlMultiDof(humanoid,chest,p.POSITION_CONTROL, targetPosition=chestRot,positionGain=kp, force=maxForce)
		p.setJointMotorControlMultiDof(humanoid,neck,p.POSITION_CONTROL,targetPosition=neckRot,positionGain=kp, force=maxForce)
		p.setJointMotorControlMultiDof(humanoid,rightHip,p.POSITION_CONTROL,targetPosition=rightHipRot,positionGain=kp, force=maxForce)
		p.setJointMotorControlMultiDof(humanoid,rightKnee,p.POSITION_CONTROL,targetPosition=rightKneeRot,positionGain=kp, force=maxForce)
		p.setJointMotorControlMultiDof(humanoid,rightAnkle,p.POSITION_CONTROL,targetPosition=rightAnkleRot,positionGain=kp, force=maxForce)
		p.setJointMotorControlMultiDof(humanoid,rightShoulder,p.POSITION_CONTROL,targetPosition=rightShoulderRot,positionGain=kp, force=maxForce)
		p.setJointMotorControlMultiDof(humanoid,rightElbow, p.POSITION_CONTROL,targetPosition=rightElbowRot,positionGain=kp, force=maxForce)
		p.setJointMotorControlMultiDof(humanoid,leftHip, p.POSITION_CONTROL,targetPosition=leftHipRot,positionGain=kp, force=maxForce)
		p.setJointMotorControlMultiDof(humanoid,leftKnee, p.POSITION_CONTROL,targetPosition=leftKneeRot,positionGain=kp, force=maxForce)
		p.setJointMotorControlMultiDof(humanoid,leftAnkle, p.POSITION_CONTROL,targetPosition=leftAnkleRot,positionGain=kp, force=maxForce)
		p.setJointMotorControlMultiDof(humanoid,leftShoulder, p.POSITION_CONTROL,targetPosition=leftShoulderRot,positionGain=kp, force=maxForce)
		p.setJointMotorControlMultiDof(humanoid,leftElbow, p.POSITION_CONTROL,targetPosition=leftElbowRot,positionGain=kp, force=maxForce)
	if (useMotionCaptureReset):
		p.resetJointStateMultiDof(humanoid,chest,chestRot)
		p.resetJointStateMultiDof(humanoid,neck,neckRot)
		p.resetJointStateMultiDof(humanoid,rightHip,rightHipRot)
		p.resetJointStateMultiDof(humanoid,rightKnee,rightKneeRot)
		p.resetJointStateMultiDof(humanoid,rightAnkle,rightAnkleRot)
		p.resetJointStateMultiDof(humanoid,rightShoulder,rightShoulderRot)
		p.resetJointStateMultiDof(humanoid,rightElbow, rightElbowRot)
		p.resetJointStateMultiDof(humanoid,leftHip, leftHipRot)
		p.resetJointStateMultiDof(humanoid,leftKnee, leftKneeRot)
		p.resetJointStateMultiDof(humanoid,leftAnkle, leftAnkleRot)
		p.resetJointStateMultiDof(humanoid,leftShoulder, leftShoulderRot)
		p.resetJointStateMultiDof(humanoid,leftElbow, leftElbowRot)
	p.stepSimulation()
	#time.sleep(1./240.)
