import pybullet as p
import time
import math

p.connect(p.PhysX)#GUI)
p.loadPlugin("eglRendererPlugin")

p.loadURDF("plane.urdf")

for i in range (10):
  sphere = p.loadURDF("marble_cube.urdf",[0,-1,1+i*1], useMaximalCoordinates=False)
p.changeDynamics(sphere ,-1, mass=1000)

door = p.loadURDF("door.urdf",[0,0,1])
p.changeDynamics(door ,1, linearDamping=0, angularDamping=0, jointDamping=0, mass=1)
print("numJoints = ", p.getNumJoints(door))


p.setGravity(0,0,-10)
position_control = True
angle = math.pi*0.25
p.resetJointState(door,1,angle)  

prevTime = time.time()

angle = math.pi*0.5

while (1):
  
  curTime = time.time()
  
  diff = curTime - prevTime
  #every second, swap target angle
  if (diff>1):
    angle = - angle
    prevTime = curTime
  
  if position_control:
    p.setJointMotorControl2(door,1,p.POSITION_CONTROL, targetPosition = angle, positionGain=10.1, velocityGain=1, force=11.001)
  else:  
    p.setJointMotorControl2(door,1,p.VELOCITY_CONTROL, targetVelocity=1, force=1011)
  p.stepSimulation()
  time.sleep(1./240.)