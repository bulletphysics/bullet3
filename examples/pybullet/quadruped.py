import pybullet as p
import time

p.connect(p.GUI)
p.loadURDF("plane.urdf")
p.loadURDF("quadruped/quadruped.urdf",0,0,0.2)
#p.getNumJoints(1)

#right front leg
p.resetJointState(1,0,1.57)
p.resetJointState(1,2,-2.2)
p.resetJointState(1,3,-1.57)
p.resetJointState(1,5,2.2)
p.createConstraint(1,2,1,5,3,[0,0,0],[0,0.01,0.2],[0,-0.015,0.2])

#left front leg
p.resetJointState(1,6,1.57)
p.resetJointState(1,8,-2.2)
p.resetJointState(1,9,-1.57)
p.resetJointState(1,11,2.2)
p.createConstraint(1,8,1,11,3,[0,0,0],[0,-0.01,0.2],[0,0.015,0.2])

#right back leg
p.resetJointState(1,12,1.57)
p.resetJointState(1,14,-2.2)
p.resetJointState(1,15,-1.57)
p.resetJointState(1,17,2.2)
p.createConstraint(1,14,1,17,3,[0,0,0],[0,0.01,0.2],[0,-0.015,0.2])

#left back leg
p.resetJointState(1,18,1.57)
p.resetJointState(1,20,-2.2)
p.resetJointState(1,21,-1.57)
p.resetJointState(1,23,2.2)
p.createConstraint(1,20,1,23,3,[0,0,0],[0,-0.01,0.2],[0,0.015,0.2])

p.setGravity(0,0,-10)
t_end = time.time() + 120 
i=0
while time.time() < t_end:
	i = p.getNumJoints(0)
	p.stepSimulation()
