import pybullet as p
import time
import math
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
#don't create a ground plane, to allow for gaps etc
p.resetSimulation()
#p.createCollisionShape(p.GEOM_PLANE)
#p.createMultiBody(0,0)
#p.resetDebugVisualizerCamera(5,75,-26,[0,0,1]);
p.resetDebugVisualizerCamera(15, -346, -16, [-15, 0, 1])

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

sphereRadius = 0.05
colSphereId = p.createCollisionShape(p.GEOM_SPHERE, radius=sphereRadius)

#a few different ways to create a mesh:

#convex mesh from obj
stoneId = p.createCollisionShape(p.GEOM_MESH, fileName="stone.obj")

boxHalfLength = 0.5
boxHalfWidth = 2.5
boxHalfHeight = 0.1
segmentLength = 5

colBoxId = p.createCollisionShape(p.GEOM_BOX,
                                  halfExtents=[boxHalfLength, boxHalfWidth, boxHalfHeight])

mass = 1
visualShapeId = -1

segmentStart = 0

for i in range(segmentLength):
  p.createMultiBody(baseMass=0,
                    baseCollisionShapeIndex=colBoxId,
                    basePosition=[segmentStart, 0, -0.1])
  segmentStart = segmentStart - 1

for i in range(segmentLength):
  height = 0
  if (i % 2):
    height = 1
  p.createMultiBody(baseMass=0,
                    baseCollisionShapeIndex=colBoxId,
                    basePosition=[segmentStart, 0, -0.1 + height])
  segmentStart = segmentStart - 1

baseOrientation = p.getQuaternionFromEuler([math.pi / 2., 0, math.pi / 2.])

for i in range(segmentLength):
  p.createMultiBody(baseMass=0,
                    baseCollisionShapeIndex=colBoxId,
                    basePosition=[segmentStart, 0, -0.1])
  segmentStart = segmentStart - 1
  if (i % 2):
    p.createMultiBody(baseMass=0,
                      baseCollisionShapeIndex=colBoxId,
                      basePosition=[segmentStart, i % 3, -0.1 + height + boxHalfWidth],
                      baseOrientation=baseOrientation)

for i in range(segmentLength):
  p.createMultiBody(baseMass=0,
                    baseCollisionShapeIndex=colBoxId,
                    basePosition=[segmentStart, 0, -0.1])
  width = 4
  for j in range(width):
    p.createMultiBody(baseMass=0,
                      baseCollisionShapeIndex=stoneId,
                      basePosition=[segmentStart, 0.5 * (i % 2) + j - width / 2., 0])
  segmentStart = segmentStart - 1

link_Masses = [1]
linkCollisionShapeIndices = [colBoxId]
linkVisualShapeIndices = [-1]
linkPositions = [[0, 0, 0]]
linkOrientations = [[0, 0, 0, 1]]
linkInertialFramePositions = [[0, 0, 0]]
linkInertialFrameOrientations = [[0, 0, 0, 1]]
indices = [0]
jointTypes = [p.JOINT_REVOLUTE]
axis = [[1, 0, 0]]

baseOrientation = [0, 0, 0, 1]
for i in range(segmentLength):
  boxId = p.createMultiBody(0,
                            colSphereId,
                            -1, [segmentStart, 0, -0.1],
                            baseOrientation,
                            linkMasses=link_Masses,
                            linkCollisionShapeIndices=linkCollisionShapeIndices,
                            linkVisualShapeIndices=linkVisualShapeIndices,
                            linkPositions=linkPositions,
                            linkOrientations=linkOrientations,
                            linkInertialFramePositions=linkInertialFramePositions,
                            linkInertialFrameOrientations=linkInertialFrameOrientations,
                            linkParentIndices=indices,
                            linkJointTypes=jointTypes,
                            linkJointAxis=axis)
  p.changeDynamics(boxId, -1, spinningFriction=0.001, rollingFriction=0.001, linearDamping=0.0)
  print(p.getNumJoints(boxId))
  for joint in range(p.getNumJoints(boxId)):
    targetVelocity = 10
    if (i % 2):
      targetVelocity = -10
    p.setJointMotorControl2(boxId,
                            joint,
                            p.VELOCITY_CONTROL,
                            targetVelocity=targetVelocity,
                            force=100)
  segmentStart = segmentStart - 1.1

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
while (1):
  camData = p.getDebugVisualizerCamera()
  viewMat = camData[2]
  projMat = camData[3]
  p.getCameraImage(256,
                   256,
                   viewMatrix=viewMat,
                   projectionMatrix=projMat,
                   renderer=p.ER_BULLET_HARDWARE_OPENGL)
  keys = p.getKeyboardEvents()
  p.stepSimulation()
  #print(keys)
  time.sleep(0.01)
