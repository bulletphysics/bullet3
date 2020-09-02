import pybullet as p
physicsClient = p.connect(p.GUI)
import pybullet_data

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

gravZ=-10
p.setGravity(0, 0, gravZ)

planeOrn = [0,0,0,1]
planeId = p.loadURDF("plane.urdf", [0,0,-2],planeOrn)

def _load_softbody(basePos):
    return p.loadSoftBody("cloth_z_up.obj", basePosition = basePos, scale = 0.5, mass = 1., useNeoHookean = 0, useBendingSprings=1,useMassSpring=1, springElasticStiffness=40, springDampingStiffness=.1, springDampingAllDirections = 1, useSelfCollision = 0, frictionCoeff = .5, useFaceContact=1, collisionMargin = 0.04)
    
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)

#load two objects and step
cloth1 = _load_softbody([0,0,1])
cloth2 = _load_softbody([0,1,0])

for i in range(300):
    p.stepSimulation()
 
# remove one object, add two and then step
p.removeBody(cloth2)
_load_softbody([0,1,1])
_load_softbody([0,-1,1])

for i in range(300):
   p.stepSimulation()

# reset simulation, add objects then step
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
p.setGravity(0, 0, gravZ)
planeId = p.loadURDF("plane.urdf", [0,0,-2],planeOrn)
_load_softbody([0,1,0])
_load_softbody([0,1,1])

while p.isConnected():
    p.stepSimulation()
