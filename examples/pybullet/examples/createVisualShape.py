import pybullet as p
import time
p.connect(p.GUI)
p.setPhysicsEngineParameter(numSolverIterations=10)
p.setTimeStep(1./120.)

logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "visualShapeBench.json")
#useMaximalCoordinates is much faster then the default reduced coordinates (Featherstone)
p.loadURDF("plane100.urdf", useMaximalCoordinates=True)
#disable rendering during creation.
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
#disable tinyrenderer, software (CPU) renderer, we don't use it here
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER,0)

shift = [0,-0.02,0]
meshScale=[0.1,0.1,0.1]
visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,fileName="duck_smooth.obj", rgbaColor=[1,1,1,1], specularColor=[0.4,.4,0], visualFramePosition=shift, meshScale=meshScale)
#boxHalfWidth=.5
#boxHalfHeight=.5
#boxHalfLength=.5
#collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[boxHalfLength,boxHalfWidth,boxHalfHeight])
collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName="duck_vhacd.obj", collisionFramePosition=shift,meshScale=meshScale)

rangex = 40
rangey = 40
for i in range (rangex):
	for j in range (rangey ):
		p.createMultiBody(baseMass=1,baseInertialFramePosition=[0,0,0],baseCollisionShapeIndex=collisionShapeId, baseVisualShapeIndex = visualShapeId, basePosition = [((-rangex/2)+i)*meshScale[0]*2,(-rangey/2+j)*meshScale[1]*2,1], useMaximalCoordinates=True)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
p.stopStateLogging(logId)
p.setGravity(0,0,-10)
p.setRealTimeSimulation(1)
while (1):
	p.setGravity(0,0,-10)
	time.sleep(1)
