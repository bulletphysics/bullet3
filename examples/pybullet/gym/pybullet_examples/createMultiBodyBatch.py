import pybullet as p
import time
import math
import pybullet_data
cid = p.connect(p.SHARED_MEMORY)
if (cid < 0):
  p.connect(p.GUI, options="--minGraphicsUpdateTimeMs=16000")
p.setAdditionalSearchPath(pybullet_data.getDataPath())  

p.setPhysicsEngineParameter(numSolverIterations=4, minimumSolverIslandSize=1024)
p.setTimeStep(1. / 120.)
logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "createMultiBodyBatch.json")
#useMaximalCoordinates is much faster then the default reduced coordinates (Featherstone)
p.loadURDF("plane100.urdf", useMaximalCoordinates=True)
#disable rendering during creation.
p.setPhysicsEngineParameter(contactBreakingThreshold=0.04)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
#disable tinyrenderer, software (CPU) renderer, we don't use it here
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)

shift = [0, -0.02, 0]
meshScale = [0.1, 0.1, 0.1]

vertices = [[-1.000000, -1.000000, 1.000000], [1.000000, -1.000000, 1.000000],
            [1.000000, 1.000000, 1.000000], [-1.000000, 1.000000, 1.000000],
            [-1.000000, -1.000000, -1.000000], [1.000000, -1.000000, -1.000000],
            [1.000000, 1.000000, -1.000000], [-1.000000, 1.000000, -1.000000],
            [-1.000000, -1.000000, -1.000000], [-1.000000, 1.000000, -1.000000],
            [-1.000000, 1.000000, 1.000000], [-1.000000, -1.000000, 1.000000],
            [1.000000, -1.000000, -1.000000], [1.000000, 1.000000, -1.000000],
            [1.000000, 1.000000, 1.000000], [1.000000, -1.000000, 1.000000],
            [-1.000000, -1.000000, -1.000000], [-1.000000, -1.000000, 1.000000],
            [1.000000, -1.000000, 1.000000], [1.000000, -1.000000, -1.000000],
            [-1.000000, 1.000000, -1.000000], [-1.000000, 1.000000, 1.000000],
            [1.000000, 1.000000, 1.000000], [1.000000, 1.000000, -1.000000]]

normals = [[0.000000, 0.000000, 1.000000], [0.000000, 0.000000, 1.000000],
           [0.000000, 0.000000, 1.000000], [0.000000, 0.000000, 1.000000],
           [0.000000, 0.000000, -1.000000], [0.000000, 0.000000, -1.000000],
           [0.000000, 0.000000, -1.000000], [0.000000, 0.000000, -1.000000],
           [-1.000000, 0.000000, 0.000000], [-1.000000, 0.000000, 0.000000],
           [-1.000000, 0.000000, 0.000000], [-1.000000, 0.000000, 0.000000],
           [1.000000, 0.000000, 0.000000], [1.000000, 0.000000, 0.000000],
           [1.000000, 0.000000, 0.000000], [1.000000, 0.000000, 0.000000],
           [0.000000, -1.000000, 0.000000], [0.000000, -1.000000, 0.000000],
           [0.000000, -1.000000, 0.000000], [0.000000, -1.000000, 0.000000],
           [0.000000, 1.000000, 0.000000], [0.000000, 1.000000, 0.000000],
           [0.000000, 1.000000, 0.000000], [0.000000, 1.000000, 0.000000]]

uvs = [[0.750000, 0.250000], [1.000000, 0.250000], [1.000000, 0.000000], [0.750000, 0.000000],
       [0.500000, 0.250000], [0.250000, 0.250000], [0.250000, 0.000000], [0.500000, 0.000000],
       [0.500000, 0.000000], [0.750000, 0.000000], [0.750000, 0.250000], [0.500000, 0.250000],
       [0.250000, 0.500000], [0.250000, 0.250000], [0.000000, 0.250000], [0.000000, 0.500000],
       [0.250000, 0.500000], [0.250000, 0.250000], [0.500000, 0.250000], [0.500000, 0.500000],
       [0.000000, 0.000000], [0.000000, 0.250000], [0.250000, 0.250000], [0.250000, 0.000000]]
indices = [
    0,
    1,
    2,
    0,
    2,
    3,  #//ground face
    6,
    5,
    4,
    7,
    6,
    4,  #//top face
    10,
    9,
    8,
    11,
    10,
    8,
    12,
    13,
    14,
    12,
    14,
    15,
    18,
    17,
    16,
    19,
    18,
    16,
    20,
    21,
    22,
    20,
    22,
    23
]

#p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER,0)
#the visual shape and collision shape can be re-used by all createMultiBody instances (instancing)
visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,
                                    rgbaColor=[1, 1, 1, 1],
                                    specularColor=[0.4, .4, 0],
                                    visualFramePosition=shift,
                                    meshScale=meshScale,
                                    vertices=vertices,
                                    indices=indices,
                                    uvs=uvs,
                                    normals=normals)
collisionShapeId = p.createCollisionShape(
    shapeType=p.GEOM_BOX, halfExtents=meshScale
)  #MESH, vertices=vertices, collisionFramePosition=shift,meshScale=meshScale)

texUid = p.loadTexture("tex256.png")

batchPositions = []

for x in range(32):
  for y in range(32):
    for z in range(10):
      batchPositions.append(
          [x * meshScale[0] * 5.5, y * meshScale[1] * 5.5, (0.5 + z) * meshScale[2] * 2.5])

bodyUids = p.createMultiBody(baseMass=0,
                            baseInertialFramePosition=[0, 0, 0],
                            baseCollisionShapeIndex=collisionShapeId,
                            baseVisualShapeIndex=visualShapeId,
                            basePosition=[0, 0, 2],
                            batchPositions=batchPositions,
                            useMaximalCoordinates=True)
p.changeVisualShape(bodyUids[0], -1, textureUniqueId=texUid)

p.syncBodyInfo()
print("numBodies=", p.getNumBodies())
p.stopStateLogging(logId)
p.setGravity(0, 0, -10)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

colors = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1], [1, 1, 1, 1]]
currentColor = 0

while (1):
  p.stepSimulation()
  #time.sleep(1./120.)
  #p.getCameraImage(320,200)
