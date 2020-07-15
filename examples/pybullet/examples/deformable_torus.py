import pybullet as p
from time import sleep
import pybullet_data

physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

p.setGravity(0, 0, -10)

planeId = p.loadURDF("plane.urdf", [0,0,-2])

boxId = p.loadURDF("cube.urdf", [0,3,2],useMaximalCoordinates = True)

bunnyId = p.loadSoftBody("torus.vtk", mass = 1, useNeoHookean = 1, NeoHookeanMu = 60, NeoHookeanLambda = 200, NeoHookeanDamping = 0.01, collisionMargin = 0.006, useSelfCollision = 1, frictionCoeff = 0.5, repulsionStiffness = 800)

bunny2 = p.loadURDF("torus_deform.urdf", [0,1,0], flags=p.URDF_USE_SELF_COLLISION)

p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
p.setRealTimeSimulation(1)

while p.isConnected():
  p.setGravity(0,0,-10)
