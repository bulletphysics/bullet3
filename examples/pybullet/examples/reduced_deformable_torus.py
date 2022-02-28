import pybullet as p
from time import sleep
import pybullet_data

physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.resetSimulation(p.RESET_USE_REDUCED_DEFORMABLE_WORLD)
p.resetDebugVisualizerCamera(4,-40,-30,[0, 0, 0])
p.setGravity(0, 0, -10)

tex = p.loadTexture("uvmap.png")
planeId = p.loadURDF("plane.urdf", [0,0,-2])

box1 = p.loadURDF("cube.urdf", [1,1,3],useMaximalCoordinates = True)
box2 = p.loadURDF("cube.urdf", [0,3,2],useMaximalCoordinates = True)

# p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "reduced_torus.mp4")
reduced_obj1= p.loadURDF("reduced_torus/reduced_torus.urdf", [1,1,1])
p.changeVisualShape(reduced_obj1, -1, rgbaColor=[1,1,1,1], textureUniqueId=tex, flags=0)

reduced_obj2 = p.loadURDF("reduced_torus/reduced_torus.urdf", [1,2,1])
p.changeVisualShape(reduced_obj2, -1, rgbaColor=[1,1,1,1], textureUniqueId=tex, flags=0)

p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
p.setRealTimeSimulation(0)

while p.isConnected():
  p.stepSimulation()
  p.getCameraImage(320,200)
  p.setGravity(0,0,-10)
