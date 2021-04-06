import pybullet as p
import time
import numpy as np

import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
p.loadURDF("sphere2.urdf",[0,0,2])

dt = 1./240.
p.setTimeStep(dt)

def getSceneAABB():
  aabbMins=[]
  aabbMaxs=[]

  for i in range(p.getNumBodies()):
    uid = p.getBodyUniqueId(i)
    aabb = p.getAABB(uid)
    aabbMins.append(np.array(aabb[0]))
    aabbMaxs.append(np.array(aabb[1]))

  if len(aabbMins):
    sceneAABBMin = aabbMins[0]
    sceneAABBMax = aabbMaxs[0]

    for aabb in aabbMins:
      sceneAABBMin = np.minimum(sceneAABBMin,aabb)
    for aabb in aabbMaxs:
      sceneAABBMax = np.maximum(sceneAABBMax,aabb)

    print("sceneAABBMin=",sceneAABBMin)
    print("sceneAABBMax=",sceneAABBMax)
 
getSceneAABB()

while (1):
  p.stepSimulation()
  time.sleep(dt)

