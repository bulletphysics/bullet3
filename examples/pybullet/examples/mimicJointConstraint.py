#a mimic joint can act as a gear between two joints
#you can control the gear ratio in magnitude and sign (>0 reverses direction)

import pybullet as p
import time
p.connect(p.GUI)
wheelA = p.loadURDF("wheel.urdf",[0,0,0])
wheelB = p.loadURDF("wheel.urdf",[0,0,1])
p.setJointMotorControl2(wheelA,0,p.VELOCITY_CONTROL,targetVelocity=0,force=0)
p.setJointMotorControl2(wheelB,0,p.VELOCITY_CONTROL,targetVelocity=1,force=1)

c = p.createConstraint(wheelA,0,wheelB,0,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
p.changeConstraint(c,gearRatio=-0.1, maxForce=10000)

p.setRealTimeSimulation(1)
while(1):
	time.sleep(0.01)
#p.removeConstraint(c)
	