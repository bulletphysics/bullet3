import pybullet as p
from time import sleep

physicsClient = p.connect(p.GUI)

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

p.setGravity(0, 0, -10)

planeId = p.loadURDF("plane.urdf", [0,0,-2])

boxId = p.loadURDF("cube.urdf", [0,3,2],useMaximalCoordinates = True)

bunnyId = p.loadSoftBody("torus.vtk", useNeoHookean = 1, NeoHookeanMu = 60, NeoHookeanLambda = 200, NeoHookeanDamping = 0.01, useSelfCollision = 1, frictionCoeff = 0.5)
p.setGravity(0, 0, -10)
while p.isConnected():
  p.stepSimulation()
