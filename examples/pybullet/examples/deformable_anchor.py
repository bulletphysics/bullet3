import pybullet as p
from time import sleep

physicsClient = p.connect(p.GUI)
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

gravZ=-10
p.setGravity(0, 0, gravZ)

planeOrn = [0,0,0,1]#p.getQuaternionFromEuler([0.3,0,0])
planeId = p.loadURDF("plane.urdf", [0,0,-2],planeOrn)

boxId = p.loadURDF("cube.urdf", [0,1,2],useMaximalCoordinates = True)

clothId = p.loadSoftBody("cloth_z_up.obj", basePosition = [0,0,2], scale = 0.5, mass = 1., useNeoHookean = 0, useBendingSprings=1,useMassSpring=1, springElasticStiffness=40, springDampingStiffness=.1, useSelfCollision = 0, frictionCoeff = .5, useFaceContact=1)


p.createSoftBodyAnchor(clothId  ,0,-1,-1)
p.createSoftBodyAnchor(clothId ,1,-1,-1)
p.createSoftBodyAnchor(clothId ,3,boxId,-1, [0.5,-0.5,0])
p.createSoftBodyAnchor(clothId ,2,boxId,-1, [-0.5,-0.5,0])
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
p.setRealTimeSimulation(1)


while p.isConnected():
  p.setGravity(0,0,gravZ)
  sleep(1./240.)
  
