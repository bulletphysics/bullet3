import pybullet as p
import time
from pybullet_utils import urdfEditor



      
      
##########################################
org2 = p.connect(p.DIRECT)
org = p.connect(p.SHARED_MEMORY)
if (org<0):
	org = p.connect(p.DIRECT)
	
gui = p.connect(p.GUI)

p.resetSimulation(physicsClientId=org)

#door.urdf, hinge.urdf, duck_vhacd.urdf, r2d2.urdf, quadruped/quadruped.urdf



mb = p.loadURDF("r2d2.urdf", flags=p.URDF_USE_IMPLICIT_CYLINDER, physicsClientId=org)
for i in range(p.getNumJoints(mb,physicsClientId=org)):
	p.setJointMotorControl2(mb,i,p.VELOCITY_CONTROL,force=0,physicsClientId=org)
	
#print("numJoints:",p.getNumJoints(mb,physicsClientId=org))

#print("base name:",p.getBodyInfo(mb,physicsClientId=org))

#for i in range(p.getNumJoints(mb,physicsClientId=org)):
#	print("jointInfo(",i,"):",p.getJointInfo(mb,i,physicsClientId=org))
#	print("linkState(",i,"):",p.getLinkState(mb,i,physicsClientId=org))

parser = urdfEditor.UrdfEditor()
parser.initializeFromBulletBody(mb,physicsClientId=org)
parser.saveUrdf("test.urdf")

if (1):
	print("\ncreatingMultiBody...................n")

	obUid = parser.createMultiBody(physicsClientId=gui)

	parser2 = urdfEditor.UrdfEditor()
	print("\n###########################################\n")
	
	parser2.initializeFromBulletBody(obUid,physicsClientId=gui)
	parser2.saveUrdf("test2.urdf")


	for i in range (p.getNumJoints(obUid, physicsClientId=gui)):
		p.setJointMotorControl2(obUid,i,p.VELOCITY_CONTROL,targetVelocity=0,force=1,physicsClientId=gui)
		print(p.getJointInfo(obUid,i,physicsClientId=gui))


	parser=0

p.setRealTimeSimulation(1,physicsClientId=gui)


while (p.getConnectionInfo(physicsClientId=gui)["isConnected"]):
	p.stepSimulation(physicsClientId=gui)
	time.sleep(0.01)
		
