import pybullet as p
import time

p.connect(p.GUI)
p.setPhysicsEngineParameter(allowedCcdPenetration=0.0)


terrain_mass=0
terrain_visual_shape_id=-1
terrain_position=[0,0,0]
terrain_orientation=[0,0,0,1]
terrain_collision_shape_id = p.createCollisionShape(
	shapeType=p.GEOM_MESH,
  fileName="terrain.obj",
  flags=p.GEOM_FORCE_CONCAVE_TRIMESH|p.GEOM_CONCAVE_INTERNAL_EDGE,
  meshScale=[0.5, 0.5, 0.5])
p.createMultiBody(
	terrain_mass, terrain_collision_shape_id, terrain_visual_shape_id,
	terrain_position, terrain_orientation)
            
            
useMaximalCoordinates = True
sphereRadius = 0.005
colSphereId = p.createCollisionShape(p.GEOM_SPHERE,radius=sphereRadius)
colBoxId = p.createCollisionShape(p.GEOM_BOX,halfExtents=[sphereRadius,sphereRadius,sphereRadius])

mass = 1
visualShapeId = -1


for i in range (5):
	for j in range (5):
		for k in range (5):
			#if (k&2):
			sphereUid = p.createMultiBody(mass,colSphereId,visualShapeId,[-i*5*sphereRadius,j*5*sphereRadius,k*2*sphereRadius+1],useMaximalCoordinates=useMaximalCoordinates)
			#else:
			#	sphereUid = p.createMultiBody(mass,colBoxId,visualShapeId,[-i*2*sphereRadius,j*2*sphereRadius,k*2*sphereRadius+1], useMaximalCoordinates=useMaximalCoordinates)
			p.changeDynamics(sphereUid,-1,spinningFriction=0.001, rollingFriction=0.001,linearDamping=0.0)
			p.changeDynamics(sphereUid,-1,ccdSweptSphereRadius=0.002)
			


p.setGravity(0,0,-10)

pts = p.getContactPoints()
print("num points=",len(pts))
print(pts)
while (p.isConnected()):
	time.sleep(1./240.)
	p.stepSimulation()
	