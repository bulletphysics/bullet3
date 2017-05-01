import pybullet as p
import math, time

#p.connect(p.UDP,"192.168.86.100")
p.connect(p.GUI)
LOWER_LIMITS = [-.967, -2.0, -2.96, 0.19, -2.96, -2.09, -3.05]
UPPER_LIMITS = [.96, 2.0, 2.96, 2.29, 2.96, 2.09, 3.05]
JOINT_RANGE = [5.8, 4, 5.8, 4, 5.8, 4, 6]
REST_POSE = [0, 0, 0, math.pi / 2, 0, -math.pi * 0.66, 0]
JOINT_DAMP = [.1, .1, .1, .1, .1, .1, .1]
REST_JOINT_POS = [-0., -0., 0., 1.570793, 0., -1.036725, 0.000001]
MAX_FORCE = 500
KUKA_GRIPPER_REST_POS = [0., -0.011130, -0.206421, 0.205143, -0.009999, 0., -0.010055, 0.]
KUKA_GRIPPER_CLOZ_POS = [0.0, -0.047564246423083795, 0.6855956234759611, -0.7479294372303137, 0.05054599996976922, 0.0, 0.049838105678835724, 0.0]

objects = [p.loadURDF("plane.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("samurai.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("pr2_gripper.urdf", 0.500000,0.300006,0.700000,-0.000000,-0.000000,-0.000031,1.000000)]
pr2_gripper = objects[0]
print ("pr2_gripper=")
print (pr2_gripper)

jointPositions=[ 0.550569, 0.000000, 0.549657, 0.000000 ]
for jointIndex in range (p.getNumJoints(pr2_gripper)):
	p.resetJointState(pr2_gripper,jointIndex,jointPositions[jointIndex])

pr2_cid = p.createConstraint(pr2_gripper,-1,-1,-1,p.JOINT_FIXED,[0,0,0],[0.2,0,0],[0.500000,0.300006,0.700000])
print ("pr2_cid")
print (pr2_cid)

objects = [p.loadURDF("kuka_iiwa/model_vr_limits.urdf", 1.400000,-0.200000,0.600000,0.000000,0.000000,0.000000,1.000000)]
kuka = objects[0]
jointPositions=[ -0.000000, -0.000000, 0.000000, 1.570793, 0.000000, -1.036725, 0.000001 ]
for jointIndex in range (p.getNumJoints(kuka)):
	p.resetJointState(kuka,jointIndex,jointPositions[jointIndex])
	p.setJointMotorControl2(kuka,jointIndex,p.POSITION_CONTROL,jointPositions[jointIndex],0)

objects = [p.loadURDF("lego/lego.urdf", 1.000000,-0.200000,0.700000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("lego/lego.urdf", 1.000000,-0.200000,0.800000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("lego/lego.urdf", 1.000000,-0.200000,0.900000,0.000000,0.000000,0.000000,1.000000)]
objects = p.loadSDF("gripper/wsg50_one_motor_gripper_new_free_base.sdf")
kuka_gripper = objects[0]
print ("kuka gripper=")
print(kuka_gripper)

p.resetBasePositionAndOrientation(kuka_gripper,[0.923103,-0.200000,1.250036],[-0.000000,0.964531,-0.000002,-0.263970])
jointPositions=[ 0.000000, -0.011130, -0.206421, 0.205143, -0.009999, 0.000000, -0.010055, 0.000000 ]
for jointIndex in range (p.getNumJoints(kuka_gripper)):
	p.resetJointState(kuka_gripper,jointIndex,jointPositions[jointIndex])
	p.setJointMotorControl2(kuka_gripper,jointIndex,p.POSITION_CONTROL,jointPositions[jointIndex],0)


kuka_cid = p.createConstraint(kuka,   6,  kuka_gripper,0,p.JOINT_FIXED, [0,0,0], [0,0,0.05],[0,0,0])

objects = [p.loadURDF("jenga/jenga.urdf", 1.300000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]
objects = [p.loadURDF("jenga/jenga.urdf", 1.200000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]
objects = [p.loadURDF("jenga/jenga.urdf", 1.100000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]
objects = [p.loadURDF("jenga/jenga.urdf", 1.000000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]
objects = [p.loadURDF("jenga/jenga.urdf", 0.900000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]
objects = [p.loadURDF("jenga/jenga.urdf", 0.800000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]
objects = [p.loadURDF("table/table.urdf", 1.000000,-0.200000,0.000000,0.000000,0.000000,0.707107,0.707107)]
objects = [p.loadURDF("teddy_vhacd.urdf", 1.050000,-0.500000,0.700000,0.000000,0.000000,0.707107,0.707107)]
objects = [p.loadURDF("cube_small.urdf", 0.950000,-0.100000,0.700000,0.000000,0.000000,0.707107,0.707107)]
objects = [p.loadURDF("sphere_small.urdf", 0.850000,-0.400000,0.700000,0.000000,0.000000,0.707107,0.707107)]
objects = [p.loadURDF("duck_vhacd.urdf", 0.850000,-0.400000,0.900000,0.000000,0.000000,0.707107,0.707107)]
objects = p.loadSDF("kiva_shelf/model.sdf")
ob = objects[0]
p.resetBasePositionAndOrientation(ob,[0.000000,1.000000,1.204500],[0.000000,0.000000,0.000000,1.000000])
objects = [p.loadURDF("teddy_vhacd.urdf", -0.100000,0.600000,0.850000,0.000000,0.000000,0.000000,1.000000)]
ball = p.loadURDF("sphere_small.urdf", -0.100000,0.955006,1.169706,0.633232,-0.000000,-0.000000,0.773962)

ball_cid = p.createConstraint(ball,-1,-1,-1,p.JOINT_FIXED,[0,0,0],[0.1,0,0],[0.00000,0.00006,0.00000])

objects = [p.loadURDF("cube_small.urdf", 0.300000,0.600000,0.850000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("table_square/table_square.urdf", -1.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
ob = objects[0]
jointPositions=[ 0.000000 ]
for jointIndex in range (p.getNumJoints(ob)):
	p.resetJointState(ob,jointIndex,jointPositions[jointIndex])

objects = [p.loadURDF("husky/husky.urdf", 2.000000,-5.000000,1.000000,0.000000,0.000000,0.000000,1.000000)]
ob = objects[0]
jointPositions=[ 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000 ]
for jointIndex in range (p.getNumJoints(ob)):
	p.resetJointState(ob,jointIndex,jointPositions[jointIndex])

p.setGravity(0,0,-10)

p.setRealTimeSimulation(1)

eef_pos_x, eef_pos_y, eef_pos_z = p.getLinkState(kuka, 6)[0]

while True:

	# p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, 'try.mp4')
	events = p.getKeyboardEvents()

	# print(events)
	# 	# A simplistic version of gripper control
	# 	#@TO-DO: Add slider for the gripper
	# 	if e[BUTTONS][33] & p.VR_BUTTON_WAS_TRIGGERED:
	# 		# avg = 0.
	# 		for i in range(p.getNumJoints(kuka_gripper)):
	# 			p.setJointMotorControl2(kuka_gripper, i, p.POSITION_CONTROL, targetPosition=KUKA_GRIPPER_CLOZ_POS[i], force=50)

	# 	if e[BUTTONS][33] & p.VR_BUTTON_WAS_RELEASED:	
	# 		for i in range(p.getNumJoints(kuka_gripper)):
	# 			p.setJointMotorControl2(kuka_gripper, i, p.POSITION_CONTROL, targetPosition=KUKA_GRIPPER_REST_POS[i], force=50)

	p.changeConstraint(ball_cid, (eef_pos_x, eef_pos_y, eef_pos_z), [0, 0, 0, 1])

	for e in events:
		# print(e)

		# print(events[e], p.KEY_IS_DOWN)
		if 120 in events and (events[120] == p.KEY_IS_DOWN):
			if e == 65297 and (events[e] == p.KEY_IS_DOWN):
				eef_pos_x += 0.01
			
			elif e == 65298 and (events[e] == p.KEY_IS_DOWN):
				eef_pos_x -= 0.01

		if 121 in events and (events[121] == p.KEY_IS_DOWN):

			if e == 65296 and (events[e] == p.KEY_IS_DOWN):
				eef_pos_y += 0.01

			if e == 65295 and (events[e] == p.KEY_IS_DOWN):
				eef_pos_y -= 0.01	

		if 122 in events and (events[122] == p.KEY_IS_DOWN):
			if e == 65297 and (events[e] == p.KEY_IS_DOWN):
				eef_pos_z += 0.01 		

			if e == 65298 and (events[e] == p.KEY_IS_DOWN):
				eef_pos_z -= 0.01
			
		joint_pos = p.calculateInverseKinematics(kuka, 6, (eef_pos_x, 
				eef_pos_y, eef_pos_z), (0, 1, 0, 0), lowerLimits=LOWER_LIMITS, upperLimits=UPPER_LIMITS, 
				jointRanges=JOINT_RANGE, restPoses=REST_POSE, jointDamping=JOINT_DAMP)

		for jointIndex in range(p.getNumJoints(kuka)):
			p.setJointMotorControl2(kuka, jointIndex, p.POSITION_CONTROL, 
				targetPosition=joint_pos[jointIndex], targetVelocity=0, positionGain=0.03,
					velocityGain=1.0, force=MAX_FORCE)
	time.sleep(0.01)





