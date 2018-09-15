import pybullet as p
import time
p.connect(p.GUI)
planeId = p.loadURDF("plane.urdf", useMaximalCoordinates=False)
cubeId = p.loadURDF("cube_collisionfilter.urdf", [0,0,3], useMaximalCoordinates=False)

collisionFilterGroup = 0
collisionFilterMask = 0
p.setCollisionFilterGroupMask(cubeId,-1,collisionFilterGroup,collisionFilterMask)

enableCollision = 1
p.setCollisionFilterPair(planeId, cubeId,-1,-1,enableCollision )

p.setRealTimeSimulation(1)
p.setGravity(0,0,-10)
while (p.isConnected()):
	time.sleep(1./240.)
	p.setGravity(0,0,-10)
