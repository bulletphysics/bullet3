import pybullet as p
import time

useMaximalCoordinates = 0

p.connect(p.GUI)
p.loadSDF("stadium.sdf",useMaximalCoordinates=useMaximalCoordinates)
sphereRadius = 0.05
colSphereId = p.createCollisionShape(p.GEOM_SPHERE,radius=sphereRadius)
colBoxId = p.createCollisionShape(p.GEOM_BOX,halfExtents=[sphereRadius,sphereRadius,sphereRadius])

mass = 1
visualShapeId = -1


for i in range (5):
	for j in range (5):
		for k in range (5):
			if (k&2):
				sphereUid = p.createMultiBody(mass,colSphereId,visualShapeId,[-i*2*sphereRadius,j*2*sphereRadius,k*2*sphereRadius+1],useMaximalCoordinates=useMaximalCoordinates)
			else:
				sphereUid = p.createMultiBody(mass,colBoxId,visualShapeId,[-i*2*sphereRadius,j*2*sphereRadius,k*2*sphereRadius+1], useMaximalCoordinates=useMaximalCoordinates)
			p.changeDynamics(sphereUid,-1,spinningFriction=0.001, rollingFriction=0.001,linearDamping=0.0)
			


p.setGravity(0,0,-10)
p.setRealTimeSimulation(1)

while (1):
	time.sleep(0.01)
	