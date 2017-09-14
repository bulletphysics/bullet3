#os.sys.path.insert is only needed when pybullet is not installed
#but running from github repo instead
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print ("current_dir=" + currentdir)
parentdir = os.path.join(currentdir,"../gym")
os.sys.path.insert(0,parentdir)


import pybullet
import pybullet_data

import time

#choose connection method: GUI, DIRECT, SHARED_MEMORY
pybullet.connect(pybullet.GUI)
pybullet.loadURDF(os.path.join(pybullet_data.getDataPath(),"plane.urdf"),0,0,-1)
#load URDF, given a relative or absolute file+path
obj = pybullet.loadURDF(os.path.join(pybullet_data.getDataPath(),"r2d2.urdf"))

posX=0
posY=3
posZ=2
obj2 = pybullet.loadURDF(os.path.join(pybullet_data.getDataPath(),"kuka_iiwa/model.urdf"),posX,posY,posZ)

#query the number of joints of the object
numJoints = pybullet.getNumJoints(obj)

print (numJoints)

#set the gravity acceleration
pybullet.setGravity(0,0,-9.8)

#step the simulation for 5 seconds
t_end = time.time() + 5
while time.time() < t_end:
	pybullet.stepSimulation()
	posAndOrn = pybullet.getBasePositionAndOrientation(obj)
	print (posAndOrn)

print ("finished")
#remove all objects
pybullet.resetSimulation()

#disconnect from the physics server
pybullet.disconnect()

