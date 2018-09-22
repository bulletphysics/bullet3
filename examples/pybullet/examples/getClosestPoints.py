import pybullet as p
import time
p.connect(p.GUI)
useCollisionShapeQuery = False

geom = p.createCollisionShape(p.GEOM_SPHERE, radius=0.1)
geomBox = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2,0.2,0.2])
obA = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=geom,basePosition=[0.5,0,1])
baseOrientationB = p.getQuaternionFromEuler([0,0.3,0])#[0,0.5,0.5,0]
basePositionB = [1.5,0,1]
obB = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=geomBox,basePosition=basePositionB,baseOrientation=baseOrientationB )

lineWidth=3
colorRGB=[1,0,0]
lineId=p.addUserDebugLine(lineFromXYZ=[0,0,0],lineToXYZ=[0,0,0],lineColorRGB=colorRGB,lineWidth=lineWidth,lifeTime=0)
pitch=0
while (p.isConnected()):
	pitch += 0.01
	if (pitch>=3.1415*2.):
		pitch=0
		
	baseOrientationB = p.getQuaternionFromEuler([0,pitch,0])#[0,0.5,0.5,0]
	p.resetBasePositionAndOrientation(obB, basePositionB, baseOrientationB)

	if (useCollisionShapeQuery):
		pts = p.getClosestPoints(bodyA=-1, bodyB=-1, distance=100, collisionShapeA=geom,collisionShapeB=geomBox, collisionShapePositionA=[0.5,0,1],collisionShapePositionB=basePositionB, collisionShapeOrientationB=baseOrientationB)
	else:
		pts = p.getClosestPoints(bodyA=obA, bodyB=obB, distance=100)

	if len(pts)>0:
		#print(pts)	
		distance = pts[0][8]
		#print("distance=",distance)
		ptA = pts[0][5]
		ptB = pts[0][6]
		p.addUserDebugLine(lineFromXYZ=ptA,lineToXYZ=ptB,lineColorRGB=colorRGB,lineWidth=lineWidth,lifeTime=0,replaceItemUniqueId=lineId); 
	time.sleep(1./240.)
	