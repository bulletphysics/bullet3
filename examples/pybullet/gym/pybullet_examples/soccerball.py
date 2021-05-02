import pybullet as p
import pybullet_data as pd
import time
p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
offset = 0
for scale in range (1,10,1):
  ball = p.loadURDF("soccerball.urdf",[offset,0,1], globalScaling=scale*0.1)
  p.changeDynamics(ball,-1,linearDamping=0, angularDamping=0, rollingFriction=0.001, spinningFriction=0.001)
  p.changeVisualShape(ball,-1,rgbaColor=[0.8,0.8,0.8,1])
  offset += 2*scale*0.1
p.loadURDF("plane.urdf")
p.setGravity(0,0,-10)
p.setRealTimeSimulation(1)
while p.isConnected():
  #p.getCameraImage(320,200, renderer=p.ER_BULLET_HARDWARE_OPENGL)
  time.sleep(0.5)
