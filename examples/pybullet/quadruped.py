import pybullet as p
import time

p.connect(p.SHARED_MEMORY)
p.loadURDF("plane.urdf")
quadruped = p.loadURDF("quadruped/quadruped.urdf",0,0,.3)
#p.getNumJoints(1)

#right front leg
p.resetJointState(quadruped,0,1.57)
p.resetJointState(quadruped,2,-2.2)
p.resetJointState(quadruped,3,-1.57)
p.resetJointState(quadruped,5,2.2)
p.createConstraint(quadruped,2,quadruped,5,3,[0,0,0],[0,0.01,0.2],[0,-0.015,0.2])

p.setJointMotorControl(quadruped,0,p.VELOCITY_CONTROL,1,10)
p.setJointMotorControl(quadruped,1,p.VELOCITY_CONTROL,0,0)
p.setJointMotorControl(quadruped,2,p.VELOCITY_CONTROL,0,0)
p.setJointMotorControl(quadruped,3,p.VELOCITY_CONTROL,-1,10)
p.setJointMotorControl(quadruped,4,p.VELOCITY_CONTROL,0,0)
p.setJointMotorControl(quadruped,5,p.VELOCITY_CONTROL,0,0)

#left front leg
p.resetJointState(quadruped,6,1.57)
p.resetJointState(quadruped,8,-2.2)
p.resetJointState(quadruped,9,-1.57)
p.resetJointState(quadruped,11,2.2)
p.createConstraint(quadruped,8,quadruped,11,3,[0,0,0],[0,-0.01,0.2],[0,0.015,0.2])

p.setJointMotorControl(quadruped,6,p.VELOCITY_CONTROL,1,10)
p.setJointMotorControl(quadruped,7,p.VELOCITY_CONTROL,0,0)
p.setJointMotorControl(quadruped,8,p.VELOCITY_CONTROL,0,0)
p.setJointMotorControl(quadruped,9,p.VELOCITY_CONTROL,-1,10)
p.setJointMotorControl(quadruped,10,p.VELOCITY_CONTROL,0,0)
p.setJointMotorControl(quadruped,11,p.VELOCITY_CONTROL,0,0)


#right back leg
p.resetJointState(quadruped,12,1.57)
p.resetJointState(quadruped,14,-2.2)
p.resetJointState(quadruped,15,-1.57)
p.resetJointState(quadruped,17,2.2)
p.createConstraint(quadruped,14,quadruped,17,3,[0,0,0],[0,0.01,0.2],[0,-0.015,0.2])

p.setJointMotorControl(quadruped,12,p.VELOCITY_CONTROL,6,10)
p.setJointMotorControl(quadruped,13,p.VELOCITY_CONTROL,0,0)
p.setJointMotorControl(quadruped,14,p.VELOCITY_CONTROL,0,0)
p.setJointMotorControl(quadruped,15,p.VELOCITY_CONTROL,-6,10)
p.setJointMotorControl(quadruped,16,p.VELOCITY_CONTROL,0,0)
p.setJointMotorControl(quadruped,17,p.VELOCITY_CONTROL,0,0)

#left back leg
p.resetJointState(quadruped,18,1.57)
p.resetJointState(quadruped,20,-2.2)
p.resetJointState(quadruped,21,-1.57)
p.resetJointState(quadruped,23,2.2)
p.createConstraint(quadruped,20,quadruped,23,3,[0,0,0],[0,-0.01,0.2],[0,0.015,0.2])

p.setJointMotorControl(quadruped,18,p.VELOCITY_CONTROL,6,10)
p.setJointMotorControl(quadruped,19,p.VELOCITY_CONTROL,0,0)
p.setJointMotorControl(quadruped,20,p.VELOCITY_CONTROL,0,0)
p.setJointMotorControl(quadruped,21,p.VELOCITY_CONTROL,-6,10)
p.setJointMotorControl(quadruped,22,p.VELOCITY_CONTROL,0,0)
p.setJointMotorControl(quadruped,23,p.VELOCITY_CONTROL,0,0)

p.setGravity(0,0,-10)

