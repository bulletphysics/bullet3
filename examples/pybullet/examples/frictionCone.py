import pybullet as p
import time
import math

import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
useMaximalCoordinates = False

p.setGravity(0, 0, -10)
plane = p.loadURDF("plane.urdf", [0, 0, -1], useMaximalCoordinates=useMaximalCoordinates)

p.setRealTimeSimulation(0)

velocity = 1
num = 40
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)  #disable this to make it faster
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
p.setPhysicsEngineParameter(enableConeFriction=1)

for i in range(num):
  print("progress:", i, num)

  x = velocity * math.sin(2. * 3.1415 * float(i) / num)
  y = velocity * math.cos(2. * 3.1415 * float(i) / num)
  print("velocity=", x, y)
  sphere = p.loadURDF("sphere_small_zeroinertia.urdf",
                      flags=p.URDF_USE_INERTIA_FROM_FILE,
                      useMaximalCoordinates=useMaximalCoordinates)
  p.changeDynamics(sphere, -1, lateralFriction=0.02)
  #p.changeDynamics(sphere,-1,rollingFriction=10)
  p.changeDynamics(sphere, -1, linearDamping=0)
  p.changeDynamics(sphere, -1, angularDamping=0)
  p.resetBaseVelocity(sphere, linearVelocity=[x, y, 0])

  prevPos = [0, 0, 0]
  for i in range(2048):
    p.stepSimulation()
    pos = p.getBasePositionAndOrientation(sphere)[0]
    if (i & 64):
      p.addUserDebugLine(prevPos, pos, [1, 0, 0], 1)
      prevPos = pos

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

while (1):
  time.sleep(0.01)
