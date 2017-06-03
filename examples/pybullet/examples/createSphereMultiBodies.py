import pybullet as p
import time

p.connect(p.GUI)
sphereRadius = 0.05

colSphereId = p.createCollisionShape(p.GEOM_SPHERE,radius=sphereRadius)
colBoxId = p.createCollisionShape(p.GEOM_BOX,halfExtents=[sphereRadius,sphereRadius,sphereRadius])

mass = 1
visualShapeId = -1


for i in range (10):
	for j in range (10):
		for k in range (10):
			if (k&2):
				sphereUid = p.createMultiBody(mass,colSphereId,visualShapeId,[-i*2*sphereRadius,j*2*sphereRadius,k*2*sphereRadius+1])
			else:
				sphereUid = p.createMultiBody(mass,colBoxId,visualShapeId,[-i*2*sphereRadius,j*2*sphereRadius,k*2*sphereRadius+1])
				#p.changeDynamics(sphereUid,-1,spinningFriction=0.1, rollingFriction=0.1)
#p.loadSDF("stadium.sdf")
p.loadURDF("plane.urdf", useMaximalCoordinates=0)

p.setGravity(0,0,-10)
p.setRealTimeSimulation(1)

while (1):
	time.sleep(0.01)
	