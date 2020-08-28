import pybullet as p
import pybullet_data
from time import sleep

physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

gravZ = -10
p.setGravity(0, 0, gravZ)

planeOrn = [0, 0, 0, 1]  # p.getQuaternionFromEuler([0.3,0,0])
planeId = p.loadURDF("plane.urdf", [0, 0, -1], planeOrn)
boxId = p.loadURDF("cube.urdf", [0, 1.5, 1], useMaximalCoordinates=True)
box2Id = p.loadURDF("cube.urdf", [0, -1.5, 1], useMaximalCoordinates=True)

min_sES = 1
max_sES = 10

softId = p.loadSoftBody("cloth.obj", basePosition=[0, 1.5, 2], scale=.5, mass=1,
                        useNeoHookean=0, useBendingSprings=1, useMassSpring=1,
                        springElasticStiffness=1, springDampingStiffness=.9, springBendingStiffness=1,
                        springDampingAllDirections=1, useSelfCollision=1, frictionCoeff=.5, useFaceContact=1)

soft2Id = p.loadSoftBody("cloth.obj", basePosition=[0, -1.5, 2], scale=.5, mass=1,
                         useNeoHookean=0, useBendingSprings=1, useMassSpring=1,
                         springElasticStiffness=1, springDampingStiffness=.9, springBendingStiffness=1,
                         springDampingAllDirections=1, useSelfCollision=1, frictionCoeff=.5, useFaceContact=1)

p.createSoftBodyAnchor(softId, 0, -1, -1)
p.createSoftBodyAnchor(softId, 1, -1, -1)
p.createSoftBodyAnchor(soft2Id, 0, -1, -1)
p.createSoftBodyAnchor(soft2Id, 1, -1, -1)
p.createSoftBodyAnchor(softId, 3, boxId, -1, [.5, 0, .5])
p.createSoftBodyAnchor(softId, 2, boxId, -1, [-.5, 0, .5])
p.createSoftBodyAnchor(soft2Id, 3, box2Id, -1, [.5, 0, .5])
p.createSoftBodyAnchor(soft2Id, 2, box2Id, -1, [-.5, 0, .5])

count = 0
switch = 0
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
p.setRealTimeSimulation(1)
p.setGravity(0, 0, gravZ)

while p.isConnected():
    sleep(1. / 240.)
    count += 1
    if count % 480 == 0:
        print("Step: " + str(count/480))
        print("Cloth_1 springElasticStiffness: " + str(p.getDynamicsInfo(softId, -1)[12][0]))
        print("Cloth_2 springElasticStiffness: " + str(p.getDynamicsInfo(soft2Id, -1)[12][0]))
        if switch % 2 == 0:
            p.changeDynamics(softId, -1, springElasticStiffness=max_sES)
            p.changeDynamics(soft2Id, -1, springElasticStiffness=min_sES)
        else:
            p.changeDynamics(softId, -1, springElasticStiffness=min_sES)
            p.changeDynamics(soft2Id, -1, springElasticStiffness=max_sES)
        switch += 1