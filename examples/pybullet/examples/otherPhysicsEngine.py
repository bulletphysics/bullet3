import pybullet as p
import pybullet_data as pd
import time
import math
import pybullet_data

usePhysX = True
useMaximalCoordinates = True
if usePhysX:
  p.connect(p.PhysX, options="--numCores=8 --solver=pgs")
  p.loadPlugin("eglRendererPlugin")
else:
  p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setPhysicsEngineParameter(fixedTimeStep=1. / 240.,
                            numSolverIterations=4,
                            minimumSolverIslandSize=1024)
p.setPhysicsEngineParameter(contactBreakingThreshold=0.01)

p.setAdditionalSearchPath(pd.getDataPath())
#Always make ground plane maximal coordinates, to avoid performance drop in PhysX
#See https://github.com/NVIDIAGameWorks/PhysX/issues/71

p.loadURDF("plane.urdf", useMaximalCoordinates=True)  #useMaximalCoordinates)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "physx_create_dominoes.json")
jran = 50
iran = 100

num = 64
radius = 0.1
numDominoes = 0

for i in range(int(num * 50)):
  num = (radius * 2 * math.pi) / 0.08
  radius += 0.05 / float(num)
  orn = p.getQuaternionFromEuler([0, 0, 0.5 * math.pi + math.pi * 2 * i / float(num)])
  pos = [
      radius * math.cos(2 * math.pi * (i / float(num))),
      radius * math.sin(2 * math.pi * (i / float(num))), 0.03
  ]
  sphere = p.loadURDF("domino/domino.urdf", pos, orn, useMaximalCoordinates=useMaximalCoordinates)
  numDominoes += 1

pos = [pos[0], pos[1], pos[2] + 0.3]
orn = p.getQuaternionFromEuler([0, 0, -math.pi / 4.])
sphere = p.loadURDF("domino/domino.urdf", pos, orn, useMaximalCoordinates=useMaximalCoordinates)

print("numDominoes=", numDominoes)

#for j in range (20):
#    for i in range (100):
#        if (i<99):
#          sphere = p.loadURDF("domino/domino.urdf",[i*0.04,1+j*.25,0.03], useMaximalCoordinates=useMaximalCoordinates)
#        else:
#          orn = p.getQuaternionFromEuler([0,-3.14*0.24,0])
#          sphere = p.loadURDF("domino/domino.urdf",[(i-1)*0.04,1+j*.25,0.03], orn, useMaximalCoordinates=useMaximalCoordinates)

print("loaded!")

#p.changeDynamics(sphere ,-1, mass=1000)

door = p.loadURDF("door.urdf", [0, 0, -11])
p.changeDynamics(door, 1, linearDamping=0, angularDamping=0, jointDamping=0, mass=1)
print("numJoints = ", p.getNumJoints(door))

p.setGravity(0, 0, -10)
position_control = True

angle = math.pi * 0.25
p.resetJointState(door, 1, angle)
angleread = p.getJointState(door, 1)
print("angleread = ", angleread)
prevTime = time.time()

angle = math.pi * 0.5

count = 0
while (1):
  count += 1
  if (count == 12):
    p.stopStateLogging(logId)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

  curTime = time.time()

  diff = curTime - prevTime
  #every second, swap target angle
  if (diff > 1):
    angle = -angle
    prevTime = curTime

  if position_control:
    p.setJointMotorControl2(door,
                            1,
                            p.POSITION_CONTROL,
                            targetPosition=angle,
                            positionGain=10.1,
                            velocityGain=1,
                            force=11.001)
  else:
    p.setJointMotorControl2(door, 1, p.VELOCITY_CONTROL, targetVelocity=1, force=1011)
  #contacts = p.getContactPoints()
  #print("contacts=",contacts)
  p.stepSimulation()
  #time.sleep(1./240.)
