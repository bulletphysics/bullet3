## Assume you have run vr_kuka_setup and have default scene set up
# Require p.setInternalSimFlags(0) in kuka_setup
import pybullet as p
import math
p.connect(p.SHARED_MEMORY)

kuka = 3
kuka_gripper = 7
POSITION = 1
BUTTONS = 6

THRESHOLD = 1.3
LOWER_LIMITS = [-.967, -2.0, -2.96, 0.19, -2.96, -2.09, -3.05]
UPPER_LIMITS = [.96, 2.0, 2.96, 2.29, 2.96, 2.09, 3.05]
JOINT_RANGE = [5.8, 4, 5.8, 4, 5.8, 4, 6]
REST_POSE = [0, 0, 0, math.pi / 2, 0, -math.pi * 0.66, 0]
JOINT_DAMP = [.1, .1, .1, .1, .1, .1, .1]
REST_JOINT_POS = [-0., -0., 0., 1.570793, 0., -1.036725, 0.000001]
MAX_FORCE = 500

def euc_dist(posA, posB):
	dist = 0.
	for i in range(len(posA)):
		dist += (posA[i] - posB[i]) ** 2
	return dist

p.setRealTimeSimulation(1)

controllers = [e[0] for e in p.getVREvents()]

while True:

	events = p.getVREvents()
	for e in (events):

		# Only use one controller
		if e[0] == min(controllers):
			break

		sq_len = euc_dist(p.getLinkState(kuka, 6)[0], e[POSITION])

		# A simplistic version of gripper control
		if e[BUTTONS][33] & p.VR_BUTTON_WAS_TRIGGERED:
			# avg = 0.
			for i in range(p.getNumJoints(kuka_gripper)):
				p.setJointMotorControl2(kuka_gripper, i, p.VELOCITY_CONTROL, targetVelocity=5, force=50)
			# 	posTarget = 0.1 + (1 - min(0.75, e[3])) * 1.5 * math.pi * 0.29;
			# 	maxPosTarget = 0.55
			# 	correction = 0.
			# 	jointPosition = p.getJointState(kuka_gripper, i)[0]
			# 	if avg:
			# 		correction = jointPosition - avg
			# 	if jointPosition < 0:
			# 		p.resetJointState(kuka_gripper, i, 0)
			# 	if jointPosition > maxPosTarget:
			# 		p.resetJointState(kuka_gripper, i, maxPosTarget)
			# 	if avg:

			# 		p.setJointMotorControl2(kuka_gripper, i, p.POSITION_CONTROL, 
			# 			targetPosition=avg, targetVelocity=0., 
			# 			positionGain=1, velocityGain=0.5, force=50)
			# 	else:
			# 		p.setJointMotorControl2(kuka_gripper, i, p.POSITION_CONTROL, 
			# 			targetPosition=posTarget, targetVelocity=0., 
			# 			positionGain=1, velocityGain=0.5, force=50)
			# 	avg = p.getJointState(kuka_gripper, i)[0]
				

		if e[BUTTONS][33] & p.VR_BUTTON_WAS_RELEASED:	
			for i in range(p.getNumJoints(kuka_gripper)):
				p.setJointMotorControl2(kuka_gripper, i, p.VELOCITY_CONTROL, targetVelocity=-5, force=50)

		if sq_len < THRESHOLD * THRESHOLD:

			joint_pos = p.calculateInverseKinematics(kuka, 6, e[POSITION], (0, 1, 0, 0), 
				lowerLimits=LOWER_LIMITS, upperLimits=UPPER_LIMITS, 
				jointRanges=JOINT_RANGE, restPoses=REST_POSE, jointDamping=JOINT_DAMP)
			for i in range(len(joint_pos)):
				p.setJointMotorControl2(kuka, i, p.POSITION_CONTROL, 
					targetPosition=joint_pos[i], targetVelocity=0, 
					positionGain=0.6, velocityGain=1.0, force=MAX_FORCE)

		else:
			# Set back to original rest pose
			for jointIndex in range(p.getNumJoints(kuka)):
				p.setJointMotorControl2(kuka, jointIndex, p.POSITION_CONTROL, 
					REST_JOINT_POS[jointIndex], 0)



