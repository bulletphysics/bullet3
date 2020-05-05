import pybullet as p
import time
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0, 0)

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
          1 + i * 5 * sphereRadius, 1 + j * 5 * sphereRadius, 1 + k * 5 * sphereRadius + 1
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

p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)

p.getNumJoints(sphereUid)
for i in range(p.getNumJoints(sphereUid)):
  p.getJointInfo(sphereUid, i)

while (1):
  keys = p.getKeyboardEvents()
  print(keys)

  time.sleep(0.01)
