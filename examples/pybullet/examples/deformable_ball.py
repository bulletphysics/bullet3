import pybullet as p
from time import sleep

physicsClient = p.connect(p.GUI)

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

p.setGravity(0, 0, -10)

planeId = p.loadURDF("plane.urdf", [0,0,-2])

boxId = p.loadURDF("cube.urdf", [0,3,2],useMaximalCoordinates = True)

ballId = p.loadSoftBody("ball.vtk", scale = 0.5, mass = 0.1, useNeoHookean = 1, NeoHookeanMu = 20, NeoHookeanLambda = 20, NeoHookeanDamping = 0.001, useSelfCollision = 1, frictionCoeff = .5)
p.setSparseSDF(size = 0.25)
p.setRealTimeSimulation(1)

while p.isConnected():
  p.setGravity(0,0,-10)
  sleep(1./240.)
