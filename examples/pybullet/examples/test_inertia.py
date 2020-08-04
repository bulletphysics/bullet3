import pybullet as p
from time import sleep
import pybullet_data


physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
useDeformable = True
if useDeformable:
	p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

gravZ=-10
p.setGravity(0, 0, gravZ)

planeOrn = [0,0,0,1]#p.getQuaternionFromEuler([0.3,0,0])
planeId = p.loadURDF("plane.urdf", [0,0,-2],planeOrn)

boxId = p.loadURDF("cube.urdf", [0,1,2],useMaximalCoordinates = True)

clothId = p.loadSoftBody("bunny.obj", basePosition = [0,0,2], scale = 0.5, mass = 1., useNeoHookean = 0, useBendingSprings=1, useMassSpring=1, springElasticStiffness=100, springDampingStiffness=.001, useSelfCollision = 0, frictionCoeff = .5, useFaceContact=1)

p.setTimeStep(0.0005)

p.setRealTimeSimulation(1)


while p.isConnected():
  p.setGravity(0,0,gravZ)
  sleep(1./240.)
  
