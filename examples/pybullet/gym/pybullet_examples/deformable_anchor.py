import pybullet as p
from time import sleep

physicsClient = p.connect(p.GUI)
import pybullet_data

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

gravZ=-10
p.setGravity(0, 0, gravZ)

planeOrn = [0,0,0,1]#p.getQuaternionFromEuler([0.3,0,0])
#planeId = p.loadURDF("plane.urdf", [0,0,-2],planeOrn)

boxId = p.loadURDF("cube.urdf", [0,1,2],useMaximalCoordinates = True)

clothId = p.loadSoftBody("cloth_z_up.obj", basePosition = [0,0,2], scale = 0.5, mass = 1., useNeoHookean = 0, useBendingSprings=1,useMassSpring=1, springElasticStiffness=40, springDampingStiffness=.1, springDampingAllDirections = 1, useSelfCollision = 0, frictionCoeff = .5, useFaceContact=1)


p.changeVisualShape(clothId, -1, flags=p.VISUAL_SHAPE_DOUBLE_SIDED)

p.createSoftBodyAnchor(clothId  ,24,-1,-1)
p.createSoftBodyAnchor(clothId ,20,-1,-1)
p.createSoftBodyAnchor(clothId ,15,boxId,-1, [0.5,-0.5,0])
p.createSoftBodyAnchor(clothId ,19,boxId,-1, [-0.5,-0.5,0])
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)

debug = True
if debug:
  data = p.getMeshData(clothId, -1, flags=p.MESH_DATA_SIMULATION_MESH)
  print("--------------")
  print("data=",data)
  print(data[0])
  print(data[1])
  text_uid = []
  for i in range(data[0]):
      pos = data[1][i]
      uid = p.addUserDebugText(str(i), pos, textColorRGB=[1,1,1])
      text_uid.append(uid)

while p.isConnected():
  p.getCameraImage(320,200)
  
  if debug:
    data = p.getMeshData(clothId, -1, flags=p.MESH_DATA_SIMULATION_MESH)
    for i in range(data[0]):
      pos = data[1][i]
      uid = p.addUserDebugText(str(i), pos, textColorRGB=[1,1,1], replaceItemUniqueId=text_uid[i])

  p.setGravity(0,0,gravZ)
  p.stepSimulation()
  #p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
  #sleep(1./240.)
  
