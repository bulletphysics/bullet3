import pybullet as p
import time

cid = p.connect(p.SHARED_MEMORY)
if (cid<0):
	p.connect(p.GUI)
p.loadURDF("plane.urdf")
kuka = p.loadURDF("kuka_iiwa/model.urdf")
p.addUserDebugText("tip", [0,0,0.1],textColorRGB=[1,0,0],textSize=1.5,parentObjectUniqueId=kuka, parentLinkIndex=6)
p.addUserDebugLine([0,0,0],[0.1,0,0],[1,0,0],parentObjectUniqueId=kuka, parentLinkIndex=6)
p.addUserDebugLine([0,0,0],[0,0.1,0],[0,1,0],parentObjectUniqueId=kuka, parentLinkIndex=6)
p.addUserDebugLine([0,0,0],[0,0,0.1],[0,0,1],parentObjectUniqueId=kuka, parentLinkIndex=6)
p.setRealTimeSimulation(0)
angle=0
while (True):
	time.sleep(0.01)
	p.resetJointState(kuka,2,angle)
	p.resetJointState(kuka,3,angle)
	angle+=0.01