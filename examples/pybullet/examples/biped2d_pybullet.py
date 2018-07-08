import pybullet as p
import pybullet_data
import os
import time
GRAVITY = -9.8
dt = 1e-3
iters=2000

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
#p.setRealTimeSimulation(True)
p.setGravity(0,0,GRAVITY)
p.setTimeStep(dt)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1.13]
cubeStartOrientation = p.getQuaternionFromEuler([0.,0,0])
botId = p.loadURDF("biped/biped2d_pybullet.urdf",
        cubeStartPos,
        cubeStartOrientation) 
 
#disable the default velocity motors 
#and set some position control with small force to emulate joint friction/return to a rest pose
jointFrictionForce=1
for joint in range (p.getNumJoints(botId)):
	p.setJointMotorControl2(botId,joint,p.POSITION_CONTROL,force=jointFrictionForce)

#for i in range(10000):     
#     p.setJointMotorControl2(botId, 1, p.TORQUE_CONTROL, force=1098.0)
#     p.stepSimulation()
#import ipdb
#ipdb.set_trace()
import time
p.setRealTimeSimulation(1)
while (1):
    #p.stepSimulation()
    #p.setJointMotorControl2(botId, 1, p.TORQUE_CONTROL, force=1098.0)
    p.setGravity(0,0,GRAVITY)
    time.sleep(1/240.)
time.sleep(1000)
