import pybullet as p
import time
import math

def getRayFromTo(mouseX,mouseY):
	width, height, viewMat, projMat, cameraUp, camForward, horizon,vertical, _,_,dist, camTarget = p.getDebugVisualizerCamera()
	camPos = [camTarget[0] - dist*camForward[0],camTarget[1] - dist*camForward[1],camTarget[2] - dist*camForward[2]]
	farPlane = 10000
	rayForward = [(camTarget[0]-camPos[0]),(camTarget[1]-camPos[1]),(camTarget[2]-camPos[2])]
	invLen = farPlane*1./(math.sqrt(rayForward[0]*rayForward[0]+rayForward[1]*rayForward[1]+rayForward[2]*rayForward[2]))
	rayForward = [invLen*rayForward[0],invLen*rayForward[1],invLen*rayForward[2]]
	rayFrom = camPos
	oneOverWidth = float(1)/float(width)
	oneOverHeight = float(1)/float(height)
	dHor = [horizon[0] * oneOverWidth,horizon[1] * oneOverWidth,horizon[2] * oneOverWidth]
	dVer = [vertical[0] * oneOverHeight,vertical[1] * oneOverHeight,vertical[2] * oneOverHeight]
	rayToCenter=[rayFrom[0]+rayForward[0],rayFrom[1]+rayForward[1],rayFrom[2]+rayForward[2]]
	rayTo = [rayFrom[0]+rayForward[0]  - 0.5 * horizon[0] + 0.5 * vertical[0]+float(mouseX)*dHor[0]-float(mouseY)*dVer[0],
					rayFrom[1]+rayForward[1]  - 0.5 * horizon[1] + 0.5 * vertical[1]+float(mouseX)*dHor[1]-float(mouseY)*dVer[1],
					rayFrom[2]+rayForward[2]  - 0.5 * horizon[2] + 0.5 * vertical[2]+float(mouseX)*dHor[2]-float(mouseY)*dVer[2]]
	return rayFrom,rayTo

cid = p.connect(p.SHARED_MEMORY)
if (cid<0):
	p.connect(p.GUI)
p.setPhysicsEngineParameter(numSolverIterations=10)
p.setTimeStep(1./120.)
logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "visualShapeBench.json")
#useMaximalCoordinates is much faster then the default reduced coordinates (Featherstone)
p.loadURDF("plane100.urdf", useMaximalCoordinates=True)
#disable rendering during creation.
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
#disable tinyrenderer, software (CPU) renderer, we don't use it here
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER,0)

shift = [0,-0.02,0]
shift1 = [0,0.1,0]
shift2 = [0,0,0]

meshScale=[0.1,0.1,0.1]
#the visual shape and collision shape can be re-used by all createMultiBody instances (instancing)
visualShapeId = p.createVisualShapeArray(shapeTypes=[p.GEOM_MESH, p.GEOM_BOX], halfExtents=[[0,0,0],[0.1,0.1,0.1]],fileNames=["duck.obj",""], visualFramePositions=[shift1,shift2,],meshScales=[meshScale,meshScale])
collisionShapeId = p.createCollisionShapeArray(shapeTypes=[p.GEOM_MESH, p.GEOM_BOX], halfExtents=[[0,0,0],[0.1,0.1,0.1]],fileNames=["duck_vhacd.obj",""], collisionFramePositions=[shift1,shift2,],meshScales=[meshScale,meshScale])


rangex =2
rangey = 2
for i in range (rangex):
	for j in range (rangey ):
		mb = p.createMultiBody(baseMass=1,baseInertialFramePosition=[0,0,0],baseCollisionShapeIndex=collisionShapeId, baseVisualShapeIndex = visualShapeId, basePosition = [((-rangex/2)+i*2)*meshScale[0]*2,(-rangey/2+j)*meshScale[1]*4,1], useMaximalCoordinates=False)
		p.changeVisualShape(mb,-1,rgbaColor=[1,1,1,1])
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
p.stopStateLogging(logId)
p.setGravity(0,0,-10)
p.setRealTimeSimulation(1)

colors = [[1,0,0,1],[0,1,0,1],[0,0,1,1],[1,1,1,1]]
currentColor = 0

p.getCameraImage(64,64, renderer=p.ER_BULLET_HARDWARE_OPENGL)

while (1):
	
	mouseEvents = p.getMouseEvents()
	for e in mouseEvents:
		if ((e[0] == 2) and (e[3]==0) and (e[4]& p.KEY_WAS_TRIGGERED)):
			mouseX = e[1]
			mouseY = e[2]
			rayFrom,rayTo=getRayFromTo(mouseX,mouseY)
			rayInfo = p.rayTest(rayFrom,rayTo)
			#p.addUserDebugLine(rayFrom,rayTo,[1,0,0],3)
			for l in range(len(rayInfo)):
				hit = rayInfo[l]
				objectUid = hit[0]
				if (objectUid>=0):
					#p.removeBody(objectUid)
					p.changeVisualShape(objectUid,-1,rgbaColor=colors[currentColor])
					currentColor+=1
					if (currentColor>=len(colors)):
						currentColor=0
