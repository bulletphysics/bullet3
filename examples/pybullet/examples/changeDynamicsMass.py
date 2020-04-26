import pybullet as p
import time
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
cube2 = p.loadURDF("cube.urdf", [0, 0, 3], useFixedBase=True)
cube = p.loadURDF("cube.urdf", useFixedBase=True)
p.setGravity(0, 0, -10)
timeStep = 1. / 240.
p.setTimeStep(timeStep)
p.changeDynamics(cube2, -1, mass=1)
#now cube2 will have a floating base and move

while (p.isConnected()):
  p.stepSimulation()
  time.sleep(timeStep)
