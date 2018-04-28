import pybullet as p
import time

p.connect(p.GUI)

if (1):
	box_collision_shape_id = p.createCollisionShape(
		shapeType=p.GEOM_BOX,
		halfExtents=[0.01,0.01,0.055])
	box_mass=0.1
	box_visual_shape_id = -1
	box_position=[0,0.1,1]
	box_orientation=[0,0,0,1]

	p.createMultiBody(
		box_mass, box_collision_shape_id, box_visual_shape_id,
		box_position, box_orientation, useMaximalCoordinates=True)
	

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
            

p.setGravity(0,0,-10)

pts = p.getContactPoints()
print("num points=",len(pts))
print(pts)
while (p.isConnected()):
	time.sleep(1./240.)
	p.stepSimulation()
	