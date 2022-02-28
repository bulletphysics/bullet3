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

boxId = p.loadURDF("cube.urdf", [1,1,5],useMaximalCoordinates = True)

#p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "reduced_cube.mp4")
cube = p.loadURDF("reduced_cube/reduced_cube.urdf", [1,1,1])
p.changeVisualShape(cube, -1, rgbaColor=[1,1,1,1], textureUniqueId=tex, flags=0)
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
p.setRealTimeSimulation(0)

while p.isConnected():
  p.stepSimulation()
  p.getCameraImage(320,200)
  p.setGravity(0,0,-10)
