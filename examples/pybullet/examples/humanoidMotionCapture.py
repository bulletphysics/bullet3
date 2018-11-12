import pybullet as p
import json

p.connect(p.GUI)
#p.configureDebugVisualizer(p.COV_ENABLE_Y_AXIS_UP , 1)


import pybullet_data

useMotionCapture=True

p.setAdditionalSearchPath(pybullet_data.getDataPath())
path = pybullet_data.getDataPath()+"/motions/humanoid3d_backflip.txt"

p.loadURDF("plane.urdf")
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
		
p.changeVisualShape(humanoid,14,rgbaColor=[1,0,0,1])
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
p.setRealTimeSimulation(0)
while (1):
	p.getCameraImage(320,200)
	frame = int(p.readUserDebugParameter(frameId))
	frameData = motion_dict['Frames'][frame]
	#print("duration=",frameData[0])
	#print(pos=[frameData])
	basePos1 = [frameData[1],frameData[2],frameData[3]]
	baseOrn1 = [frameData[5],frameData[6], frameData[7],frameData[4]]
	#pre-rotate to make z-up
	y2zPos=[0,0,0]
	y2zOrn = p.getQuaternionFromEuler([1.57,0,0])
	basePos,baseOrn = p.multiplyTransforms(y2zPos, y2zOrn,basePos1,baseOrn1)
	p.resetBasePositionAndOrientation(humanoid, basePos,baseOrn)
	chestRot = [frameData[9],frameData[10],frameData[11],frameData[8]]
	neckRot = [frameData[13],frameData[14],frameData[15],frameData[12]]
	rightHipRot = [frameData[17],frameData[18],frameData[19],frameData[16]]
	rightKneeRot = [frameData[20]]
	rightAnkleRot = [frameData[22],frameData[23],frameData[24],frameData[21]]
	rightShoulderRot = [frameData[26],frameData[27],frameData[28],frameData[25]]
	rightElbowRot = [frameData[29]]
	leftHipRot = [frameData[31],frameData[32],frameData[33],frameData[30]]
	leftKneeRot = [frameData[34]]
	leftAnkleRot = [frameData[36],frameData[37],frameData[38],frameData[35]]
	leftShoulderRot = [frameData[40],frameData[41],frameData[42],frameData[39]]
	leftElbowRot = [frameData[43]]
	#print("chestRot=",chestRot)
	if (useMotionCapture):
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
	else:
		for i in range(len(paramIds)):
			c = paramIds[i]
			targetPos = p.readUserDebugParameter(c)
			
			p.setJointMotorControl2(humanoid,jointIds[i],
				p.POSITION_CONTROL,targetPos, force=5*240.)
	time.sleep(0.1)
