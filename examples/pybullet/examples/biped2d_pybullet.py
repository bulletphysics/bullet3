import pybullet as p
import pybullet_data
import os
import time

import pybullet as p
import pybullet_data as pd
import math
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())

textureId = -1

useProgrammatic = 0
useTerrainFromPNG = 1
useDeepLocoCSV = 2
updateHeightfield = False

heightfieldSource = useProgrammatic
import random
random.seed(10)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
heightPerturbationRange = 0.05
if heightfieldSource==useProgrammatic:
  numHeightfieldRows = 256
  numHeightfieldColumns = 256
  heightfieldData = [0]*numHeightfieldRows*numHeightfieldColumns 
  for j in range (int(numHeightfieldColumns/2)):
    for i in range (int(numHeightfieldRows/2) ):
      height = random.uniform(0,heightPerturbationRange)
      heightfieldData[2*i+2*j*numHeightfieldRows]=height
      heightfieldData[2*i+1+2*j*numHeightfieldRows]=height
      heightfieldData[2*i+(2*j+1)*numHeightfieldRows]=height
      heightfieldData[2*i+1+(2*j+1)*numHeightfieldRows]=height
      
      
  terrainShape = p.createCollisionShape(shapeType = p.GEOM_HEIGHTFIELD, meshScale=[.05,.05,1], heightfieldTextureScaling=(numHeightfieldRows-1)/2, heightfieldData=heightfieldData, numHeightfieldRows=numHeightfieldRows, numHeightfieldColumns=numHeightfieldColumns)
  terrain  = p.createMultiBody(0, terrainShape)
  p.resetBasePositionAndOrientation(terrain,[0,0,0], [0,0,0,1])

if heightfieldSource==useDeepLocoCSV:
  terrainShape = p.createCollisionShape(shapeType = p.GEOM_HEIGHTFIELD, meshScale=[.5,.5,2.5],fileName = "heightmaps/ground0.txt", heightfieldTextureScaling=128)
  terrain  = p.createMultiBody(0, terrainShape)
  p.resetBasePositionAndOrientation(terrain,[0,0,0], [0,0,0,1])

if heightfieldSource==useTerrainFromPNG:
  terrainShape = p.createCollisionShape(shapeType = p.GEOM_HEIGHTFIELD, meshScale=[.1,.1,24],fileName = "heightmaps/wm_height_out.png")
  textureId = p.loadTexture("heightmaps/gimp_overlay_out.png")
  terrain  = p.createMultiBody(0, terrainShape)
  p.changeVisualShape(terrain, -1, textureUniqueId = textureId)
 
 
p.changeVisualShape(terrain, -1, rgbaColor=[1,1,1,1])


sphereRadius = 0.05
colSphereId = p.createCollisionShape(p.GEOM_SPHERE, radius=sphereRadius)
colBoxId = p.createCollisionShape(p.GEOM_BOX,
                                  halfExtents=[sphereRadius, sphereRadius, sphereRadius])

mass = 1
visualShapeId = -1

link_Masses = [1]
linkCollisionShapeIndices = [colBoxId]
linkVisualShapeIndices = [-1]
linkPositions = [[0, 0, 0.11]]
linkOrientations = [[0, 0, 0, 1]]
linkInertialFramePositions = [[0, 0, 0]]
linkInertialFrameOrientations = [[0, 0, 0, 1]]
indices = [0]
jointTypes = [p.JOINT_REVOLUTE]
axis = [[0, 0, 1]]

for i in range(3):
  for j in range(3):
    for k in range(3):
      basePosition = [
          i * 5 * sphereRadius, j * 5 * sphereRadius, 1 + k * 5 * sphereRadius + 1
      ]
      baseOrientation = [0, 0, 0, 1]
      if (k & 2):
        sphereUid = p.createMultiBody(mass, colSphereId, visualShapeId, basePosition,
                                      baseOrientation)
      else:
        sphereUid = p.createMultiBody(mass,
                                      colBoxId,
                                      visualShapeId,
                                      basePosition,
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

      
      p.changeDynamics(sphereUid,
                       -1,
                       spinningFriction=0.001,
                       rollingFriction=0.001,
                       linearDamping=0.0)
      for joint in range(p.getNumJoints(sphereUid)):
        p.setJointMotorControl2(sphereUid, joint, p.VELOCITY_CONTROL, targetVelocity=1, force=10)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)

GRAVITY = -9.8
dt = 1e-3
iters = 2000
import pybullet_data

#physicsClient = p.connect(p.GUI)


p.setAdditionalSearchPath(pybullet_data.getDataPath())
#p.resetSimulation()
#p.setRealTimeSimulation(True)
p.setGravity(0, 0, GRAVITY)
p.setTimeStep(dt)
#planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 1.13]
cubeStartOrientation = p.getQuaternionFromEuler([0., 0, 0])
botId = p.loadURDF("biped/biped2d_pybullet.urdf", cubeStartPos, cubeStartOrientation)

#disable the default velocity motors
#and set some position control with small force to emulate joint friction/return to a rest pose
jointFrictionForce = 1
for joint in range(p.getNumJoints(botId)):
  p.setJointMotorControl2(botId, joint, p.POSITION_CONTROL, force=jointFrictionForce)

#for i in range(10000):
#     p.setJointMotorControl2(botId, 1, p.TORQUE_CONTROL, force=1098.0)
#     p.stepSimulation()
#import ipdb
#ipdb.set_trace()
import time
p.setRealTimeSimulation(1)
while (1):
  #p.stepSimulation()
  #p.setJointMotorControl2(botId, 1, p.TORQUE_CONTROL, force=1098.0)
  p.setGravity(0, 0, GRAVITY)
  time.sleep(1 / 240.)
time.sleep(1000)
