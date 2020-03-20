import pybullet as p
import time

p.connect(p.GRAPHICS_SERVER)
while p.isConnected():
  #p.stepSimulation()
  time.sleep(1./240.)