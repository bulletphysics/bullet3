import pybullet as p
p.connect(p.DIRECT)
hinge = p.loadURDF("hinge.urdf")
print("mass of linkA = 1kg, linkB = 1kg, total mass = 2kg")

hingeJointIndex = 0

#by default, joint motors are enabled, maintaining zero velocity
p.setJointMotorControl2(hinge,hingeJointIndex,p.VELOCITY_CONTROL,0,0,0)

p.setGravity(0,0,-10)
p.stepSimulation()
print("joint state without sensor")

print(p.getJointState(0,0))
p.enableJointForceTorqueSensor(hinge,hingeJointIndex)
p.stepSimulation()
print("joint state with force/torque sensor, gravity [0,0,-10]")
print(p.getJointState(0,0))
p.setGravity(0,0,0)
p.stepSimulation()
print("joint state with force/torque sensor, no gravity")
print(p.getJointState(0,0))

p.disconnect()

