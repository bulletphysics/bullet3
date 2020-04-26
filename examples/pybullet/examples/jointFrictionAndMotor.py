import pybullet as p
import time

import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
door = p.loadURDF("door.urdf")

#linear/angular damping for base and all children=0
p.changeDynamics(door, -1, linearDamping=0, angularDamping=0)
for j in range(p.getNumJoints(door)):
  p.changeDynamics(door, j, linearDamping=0, angularDamping=0)
  print(p.getJointInfo(door, j))

frictionId = p.addUserDebugParameter("jointFriction", 0, 20, 10)
torqueId = p.addUserDebugParameter("joint torque", 0, 20, 5)

while (1):
  frictionForce = p.readUserDebugParameter(frictionId)
  jointTorque = p.readUserDebugParameter(torqueId)
  #set the joint friction
  p.setJointMotorControl2(door, 1, p.VELOCITY_CONTROL, targetVelocity=0, force=frictionForce)
  #apply a joint torque
  p.setJointMotorControl2(door, 1, p.TORQUE_CONTROL, force=jointTorque)
  p.stepSimulation()
  time.sleep(0.01)
