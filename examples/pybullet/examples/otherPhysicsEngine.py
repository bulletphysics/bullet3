import pybullet as p
import pybullet_data as pd
import time
import math

usePhysX = True
if usePhysX:
  p.connect(p.PhysX,options="--numCores=1 --solver=pgs")
  p.loadPlugin("eglRendererPlugin")
else:
  p.connect(p.GUI)

useMaximalCoordinates = False
p.setAdditionalSearchPath(pd.getDataPath())
p.loadURDF("plane.urdf", useMaximalCoordinates=useMaximalCoordinates)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER,0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS,"physx_create_dominoes.json")
for j in range (50):
    for i in range (100):
        sphere = p.loadURDF("domino/domino.urdf",[i*0.04,1+j*.25,0.03], useMaximalCoordinates=useMaximalCoordinates)
print("loaded!")
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)

p.changeDynamics(sphere ,-1, mass=1000)

door = p.loadURDF("door.urdf",[0,0,1])
p.changeDynamics(door ,1, linearDamping=0, angularDamping=0, jointDamping=0, mass=1)
print("numJoints = ", p.getNumJoints(door))


p.setGravity(0,0,-10)
position_control = True

angle = math.pi*0.25
p.resetJointState(door,1,angle)  
angleread = p.getJointState(door,1)
print("angleread = ",angleread)
prevTime = time.time()

angle = math.pi*0.5

count=0
while (1):
  count+=1
  if (count==10):
      p.stopStateLogging(logId)

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
  #contacts = p.getContactPoints()
  #print("contacts=",contacts)
  p.stepSimulation()
  time.sleep(1./240.)
