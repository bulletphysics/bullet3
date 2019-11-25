import pybullet as p
from time import sleep

physicsClient = p.connect(p.GUI)

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

p.setGravity(0, 0, -10)

planeOrn = [0,0,0,1]#p.getQuaternionFromEuler([0.3,0,0])
planeId = p.loadURDF("plane.urdf", [0,0,-2],planeOrn)

boxId = p.loadURDF("cube.urdf", [0,3,2],useMaximalCoordinates = True)

ballId = p.loadSoftBody("ball.vtk", basePosition = [0,0,-1], scale = 0.5, mass = 0.1, useNeoHookean = 1, NeoHookeanMu = 20, NeoHookeanLambda = 20, NeoHookeanDamping = 0.001, useSelfCollision = 1, frictionCoeff = .5)
p.setTimeStep(0.001)
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
p.setRealTimeSimulation(1)

#logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "perf.json")

while p.isConnected():
  
  p.setGravity(0,0,-10)
  sleep(1./240.)
  
#p.resetSimulation()
#p.stopStateLogging(logId)
