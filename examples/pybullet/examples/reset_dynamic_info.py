import pybullet as p
import time
import math

import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF(fileName="plane.urdf", baseOrientation=[0.25882, 0, 0, 0.96593])
p.loadURDF(fileName="cube.urdf", basePosition=[0, 0, 2])
cubeId = p.loadURDF(fileName="cube.urdf", baseOrientation=[0, 0, 0, 1], basePosition=[0, 0, 4])
#p.changeDynamics(bodyUniqueId=2,linkIndex=-1,mass=0.1)
p.changeDynamics(bodyUniqueId=2, linkIndex=-1, mass=1.0)
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(0)


def drawInertiaBox(parentUid, parentLinkIndex):
  mass, frictionCoeff, inertia = p.getDynamicsInfo(bodyUniqueId=parentUid,
                                                   linkIndex=parentLinkIndex,
                                                   flags=p.DYNAMICS_INFO_REPORT_INERTIA)
  Ixx = inertia[0]
  Iyy = inertia[1]
  Izz = inertia[2]
  boxScaleX = 0.5 * math.sqrt(6 * (Izz + Iyy - Ixx) / mass)
  boxScaleY = 0.5 * math.sqrt(6 * (Izz + Ixx - Iyy) / mass)
  boxScaleZ = 0.5 * math.sqrt(6 * (Ixx + Iyy - Izz) / mass)

  halfExtents = [boxScaleX, boxScaleY, boxScaleZ]
  pts = [[halfExtents[0], halfExtents[1], halfExtents[2]],
         [-halfExtents[0], halfExtents[1], halfExtents[2]],
         [halfExtents[0], -halfExtents[1], halfExtents[2]],
         [-halfExtents[0], -halfExtents[1], halfExtents[2]],
         [halfExtents[0], halfExtents[1], -halfExtents[2]],
         [-halfExtents[0], halfExtents[1], -halfExtents[2]],
         [halfExtents[0], -halfExtents[1], -halfExtents[2]],
         [-halfExtents[0], -halfExtents[1], -halfExtents[2]]]

  color = [1, 0, 0]
  p.addUserDebugLine(pts[0],
                     pts[1],
                     color,
                     1,
                     parentObjectUniqueId=parentUid,
                     parentLinkIndex=parentLinkIndex)
  p.addUserDebugLine(pts[1],
                     pts[3],
                     color,
                     1,
                     parentObjectUniqueId=parentUid,
                     parentLinkIndex=parentLinkIndex)
  p.addUserDebugLine(pts[3],
                     pts[2],
                     color,
                     1,
                     parentObjectUniqueId=parentUid,
                     parentLinkIndex=parentLinkIndex)
  p.addUserDebugLine(pts[2],
                     pts[0],
                     color,
                     1,
                     parentObjectUniqueId=parentUid,
                     parentLinkIndex=parentLinkIndex)

  p.addUserDebugLine(pts[0],
                     pts[4],
                     color,
                     1,
                     parentObjectUniqueId=parentUid,
                     parentLinkIndex=parentLinkIndex)
  p.addUserDebugLine(pts[1],
                     pts[5],
                     color,
                     1,
                     parentObjectUniqueId=parentUid,
                     parentLinkIndex=parentLinkIndex)
  p.addUserDebugLine(pts[2],
                     pts[6],
                     color,
                     1,
                     parentObjectUniqueId=parentUid,
                     parentLinkIndex=parentLinkIndex)
  p.addUserDebugLine(pts[3],
                     pts[7],
                     color,
                     1,
                     parentObjectUniqueId=parentUid,
                     parentLinkIndex=parentLinkIndex)

  p.addUserDebugLine(pts[4 + 0],
                     pts[4 + 1],
                     color,
                     1,
                     parentObjectUniqueId=parentUid,
                     parentLinkIndex=parentLinkIndex)
  p.addUserDebugLine(pts[4 + 1],
                     pts[4 + 3],
                     color,
                     1,
                     parentObjectUniqueId=parentUid,
                     parentLinkIndex=parentLinkIndex)
  p.addUserDebugLine(pts[4 + 3],
                     pts[4 + 2],
                     color,
                     1,
                     parentObjectUniqueId=parentUid,
                     parentLinkIndex=parentLinkIndex)
  p.addUserDebugLine(pts[4 + 2],
                     pts[4 + 0],
                     color,
                     1,
                     parentObjectUniqueId=parentUid,
                     parentLinkIndex=parentLinkIndex)


drawInertiaBox(cubeId, -1)

t = 0
while 1:
  t = t + 1
  if t > 400:
    p.changeDynamics(bodyUniqueId=0, linkIndex=-1, lateralFriction=0.01)
  mass1, frictionCoeff1 = p.getDynamicsInfo(bodyUniqueId=planeId, linkIndex=-1)
  mass2, frictionCoeff2 = p.getDynamicsInfo(bodyUniqueId=cubeId, linkIndex=-1)

  print(mass1, frictionCoeff1)
  print(mass2, frictionCoeff2)
  time.sleep(1. / 240.)
  p.stepSimulation()
