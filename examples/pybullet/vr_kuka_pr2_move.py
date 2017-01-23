#python script with hardcoded values, assumes that you run the vr_kuka_setup.py first

import pybullet as p
p.connect(p.SHARED_MEMORY)

pr2_gripper = 2
pr2_cid = 1

CONTROLLER_ID = 0
POSITION=1
ORIENTATION=2
BUTTONS=6

while True:
	events = p.getVREvents()

	for e in (events):
		if (e[BUTTONS][33]&p.VR_BUTTON_IS_DOWN):
			p.changeConstraint(pr2_cid,e[POSITION],e[ORIENTATION], maxForce=50)
			#todo
			#p.setJointMotorControl2(pr2_gripper,0)
			