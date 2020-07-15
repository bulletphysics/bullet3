import pybullet as p
import time
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeUidA = p.loadURDF("plane_transparent.urdf", [0, 0, 0])
planeUid = p.loadURDF("plane_transparent.urdf", [0, 0, -1])

texUid = p.loadTexture("tex256.png")

p.changeVisualShape(planeUidA, -1, rgbaColor=[1, 1, 1, 0.5])
p.changeVisualShape(planeUid, -1, rgbaColor=[1, 1, 1, 0.5])
p.changeVisualShape(planeUid, -1, textureUniqueId=texUid)

width = 256
height = 256
pixels = [255] * width * height * 3
colorR = 0
colorG = 0
colorB = 0

#p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
#p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

blue = 0
logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "renderbench.json")
for i in range(100000):
  p.stepSimulation()
  for i in range(width):
    for j in range(height):
      pixels[(i + j * width) * 3 + 0] = i
      pixels[(i + j * width) * 3 + 1] = (j + blue) % 256
      pixels[(i + j * width) * 3 + 2] = blue
  blue = blue + 1
  p.changeTexture(texUid, pixels, width, height)
  start = time.time()
  p.getCameraImage(300, 300, renderer=p.ER_BULLET_HARDWARE_OPENGL)
  end = time.time()
  print("rendering duration")
  print(end - start)
p.stopStateLogging(logId)
#p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
#p.configureDebugVisualizer(p.COV_ENABLE_GUI,1)
