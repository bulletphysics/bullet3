import pybullet as p
import pybullet
import time
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("toys/concave_box.urdf")
p.setGravity(0, 0, -10)
for i in range(10):
  p.loadURDF("sphere_1cm.urdf", [i * 0.02, 0, 0.5])
p.loadURDF("duck_vhacd.urdf")
timeStep = 1. / 240.
p.setTimeStep(timeStep)
while (1):
  p.stepSimulation()
  time.sleep(timeStep)
