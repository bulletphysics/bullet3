import pybullet as p
import math
import time
dt = 1./240.
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("r2d2.urdf",[0,0,1])
p.loadURDF("plane.urdf")
p.setGravity(0,0,-10)

radius=5
t = 0
p.configureDebugVisualizer(shadowMapWorldSize=5)
p.configureDebugVisualizer(shadowMapResolution=8192)

while (1):
  t+=dt
  p.configureDebugVisualizer(lightPosition=[radius*math.sin(t),radius*math.cos(t),3])
  
  p.stepSimulation()
  time.sleep(dt)
