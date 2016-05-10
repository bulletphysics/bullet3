import pybullet
import time

#choose connection method: GUI, DIRECT, SHARED_MEMORY
pybullet.connect(pybullet.GUI)

#load URDF, given a relative or absolute file+path
obj = pybullet.loadURDF("C:/develop/bullet3/data/r2d2.urdf")

#query the number of joints of the object
numJoints = pybullet.getNumJoints(obj)

print (numJoints)

#set the gravity acceleration
pybullet.setGravity(0,0,-0.01)

#step the simulation for 5 seconds
t_end = time.time() + 5
while time.time() < t_end:
	pybullet.stepSimulation()

print ("finished")
#remove all objects
pybullet.resetSimulation()

#disconnect from the physics server
pybullet.disconnect()


