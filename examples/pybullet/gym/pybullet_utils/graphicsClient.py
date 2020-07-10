import pybullet as p
import time

p.connect(p.GRAPHICS_SERVER_TCP)
import pybullet_data as pd
p.setAdditionalSearchPath(pd.getDataPath())

p.loadURDF("plane.urdf")
p.loadURDF("r2d2.urdf", [0,0,3])
p.setGravity(0,0,-10)
while p.isConnected():
  p.stepSimulation()

