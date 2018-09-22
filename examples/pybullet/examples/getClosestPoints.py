import pybullet as p
import time
p.connect(p.GUI)
useCollisionShapeQuery = True
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
geom = p.createCollisionShape(p.GEOM_SPHERE, radius=0.1)
geomBox = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2,0.2,0.2])
baseOrientationB = p.getQuaternionFromEuler([0,0.3,0])#[0,0.5,0.5,0]
basePositionB = [1.5,0,1]
obA=-1
obB=-1

obA = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=geom,basePosition=[0.5,0,1])
obB = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=geomBox,basePosition=basePositionB,baseOrientation=baseOrientationB )


lineWidth=3
colorRGB=[1,0,0]
lineId=p.addUserDebugLine(lineFromXYZ=[0,0,0],lineToXYZ=[0,0,0],lineColorRGB=colorRGB,lineWidth=lineWidth,lifeTime=0)
pitch=0
yaw=0

while (p.isConnected()):
	pitch += 0.01
	if (pitch>=3.1415*2.):
		pitch=0
	yaw+= 0.01
	if (yaw>=3.1415*2.):
		yaw=0
		
	baseOrientationB = p.getQuaternionFromEuler([yaw,pitch,0])
	if (obB>=0):
		p.resetBasePositionAndOrientation(obB, basePositionB, baseOrientationB)

	if (useCollisionShapeQuery):
		pts = p.getClosestPoints(bodyA=-1, bodyB=-1, distance=100, collisionShapeA=geom,collisionShapeB=geomBox, collisionShapePositionA=[0.5,0,1],collisionShapePositionB=basePositionB, collisionShapeOrientationB=baseOrientationB)
		#pts = p.getClosestPoints(bodyA=obA, bodyB=-1, distance=100, collisionShapeB=geomBox, collisionShapePositionB=basePositionB, collisionShapeOrientationB=baseOrientationB)
	else:
		pts = p.getClosestPoints(bodyA=obA, bodyB=obB, distance=100)

	if len(pts)>0:
		#print(pts)	
		distance = pts[0][8]
		#print("distance=",distance)
		ptA = pts[0][5]
		ptB = pts[0][6]
		p.addUserDebugLine(lineFromXYZ=ptA,lineToXYZ=ptB,lineColorRGB=colorRGB,lineWidth=lineWidth,lifeTime=0,replaceItemUniqueId=lineId); 
	#time.sleep(1./240.)
	

#removeCollisionShape is optional:
#only use removeCollisionShape if the collision shape is not used to create a body
#and if you want to keep on creating new collision shapes for different queries (not recommended)
p.removeCollisionShape(geom)
p.removeCollisionShape(geomBox)
