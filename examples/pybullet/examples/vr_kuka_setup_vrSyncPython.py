import pybullet as p
import time
#p.connect(p.UDP,"192.168.86.100")


cid = p.connect(p.SHARED_MEMORY)
if (cid<0):
	p.connect(p.GUI)
p.resetSimulation()
#disable rendering during loading makes it much faster
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
objects = [p.loadURDF("plane.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
#objects = [p.loadURDF("samurai.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("pr2_gripper.urdf", 0.500000,0.300006,0.700000,-0.000000,-0.000000,-0.000031,1.000000)]
pr2_gripper = objects[0]
print ("pr2_gripper=")
print (pr2_gripper)

jointPositions=[ 0.550569, 0.000000, 0.549657, 0.000000 ]
for jointIndex in range (p.getNumJoints(pr2_gripper)):
	p.resetJointState(pr2_gripper,jointIndex,jointPositions[jointIndex])
	p.setJointMotorControl2(pr2_gripper,jointIndex,p.POSITION_CONTROL,targetPosition=0,force=0)

pr2_cid = p.createConstraint(pr2_gripper,-1,-1,-1,p.JOINT_FIXED,[0,0,0],[0.2,0,0],[0.500000,0.300006,0.700000])
print ("pr2_cid")
print (pr2_cid)

pr2_cid2 = p.createConstraint(pr2_gripper,0,pr2_gripper,2,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
p.changeConstraint(pr2_cid2,gearRatio=1, erp=0.5, relativePositionTarget=0.5, maxForce=3)



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
objects = [p.loadURDF("sphere_small.urdf", -0.100000,0.955006,1.169706,0.633232,-0.000000,-0.000000,0.773962)]
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

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

p.setGravity(0.000000,0.000000,0.000000)
p.setGravity(0,0,-10)

##show this for 10 seconds
#now = time.time()
#while (time.time() < now+10):
#	p.stepSimulation()
p.setRealTimeSimulation(1)

CONTROLLER_ID = 0
POSITION=1
ORIENTATION=2
ANALOG=3
BUTTONS=6


controllerId = -1

print("waiting for VR controller trigger")
while (controllerId<0):
	events = p.getVREvents()
	for e in (events):
		if (e[BUTTONS][33]==p.VR_BUTTON_IS_DOWN):
			controllerId = e[CONTROLLER_ID]
		if (e[BUTTONS][32]==p.VR_BUTTON_IS_DOWN):
			controllerId = e[CONTROLLER_ID]

print("Using controllerId="+str(controllerId))

while (1):
	

	#keep the gripper centered/symmetric
	b = p.getJointState(pr2_gripper,2)[0]
	p.setJointMotorControl2(pr2_gripper, 0, p.POSITION_CONTROL, targetPosition=b, force=3)


	events = p.getVREvents()
	for e in (events):
		if e[CONTROLLER_ID] == controllerId:  # To make sure we only get the value for one of the remotes
			#sync the vr pr2 gripper with the vr controller position
			p.changeConstraint(pr2_cid, e[POSITION], e[ORIENTATION], maxForce=500)
			relPosTarget = 1 - e[ANALOG]
			#open/close the gripper, based on analogue
			p.changeConstraint(pr2_cid2,gearRatio=1, erp=1, relativePositionTarget=relPosTarget, maxForce=3)
			