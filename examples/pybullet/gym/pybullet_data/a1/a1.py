import pybullet as p
import time
import pybullet_data as pd
p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
dt = 1./240.

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
p.loadURDF("plane.urdf")
p.loadURDF("a1.urdf",[0,0,0.5])
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
p.setGravity(0,0,-9.8)

while 1:
  p.stepSimulation()
  time.sleep(dt)


