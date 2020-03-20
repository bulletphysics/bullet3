import pybullet as p
import time

p.connect(p.GRAPHICS_SERVER)
#p.connect(p.GRAPHICS_SERVER_MAIN_THREAD)
while p.isConnected():
  p.stepSimulation()
  time.sleep(1./240.)