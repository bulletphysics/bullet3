import pybullet as p
import time
import pybullet_data

import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)
visualShift = [0, 0, 0]
collisionShift = [0, 0, 0]
inertiaShift = [0, 0, -0.5]

meshScale = [1, 1, 1]
visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,
                                    fileName="cube.obj",
                                    rgbaColor=[1, 1, 1, 1],
                                    specularColor=[0.4, .4, 0],
                                    visualFramePosition=visualShift,
                                    meshScale=meshScale)
collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                          fileName="cube.obj",
                                          collisionFramePosition=collisionShift,
                                          meshScale=meshScale)

p.createMultiBody(baseMass=1,
                  baseInertialFramePosition=inertiaShift,
                  baseCollisionShapeIndex=collisionShapeId,
                  baseVisualShapeIndex=visualShapeId,
                  basePosition=[0, 0, 1],
                  useMaximalCoordinates=False)
while (1):
  p.stepSimulation()
  time.sleep(1. / 240.)
