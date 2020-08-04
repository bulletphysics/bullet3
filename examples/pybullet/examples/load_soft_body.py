import pybullet as p
from time import sleep
import pybullet_data


physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf", [0,0,-2])
boxId = p.loadURDF("cube.urdf", [0,3,2],useMaximalCoordinates = True)
bunnyId = p.loadSoftBody("bunny.obj")#.obj")#.vtk")

#meshData = p.getMeshData(bunnyId)
#print("meshData=",meshData)
#p.loadURDF("cube_small.urdf", [1, 0, 1])
useRealTimeSimulation = 1

if (useRealTimeSimulation):
  p.setRealTimeSimulation(1)

print(p.getDynamicsInfo(planeId, -1))
#print(p.getDynamicsInfo(bunnyId, 0))
print(p.getDynamicsInfo(boxId, -1))
p.changeDynamics(boxId,-1,mass=10)
while p.isConnected():
  p.setGravity(0, 0, -10)
  if (useRealTimeSimulation):

    sleep(0.01)  # Time in seconds.
    #p.getCameraImage(320,200,renderer=p.ER_BULLET_HARDWARE_OPENGL )
  else:
    p.stepSimulation()
