import pybullet as p
import time
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
sphereUid = p.loadURDF("sphere_transparent.urdf", [0, 0, 2])

redSlider = p.addUserDebugParameter("red", 0, 1, 1)
greenSlider = p.addUserDebugParameter("green", 0, 1, 0)
blueSlider = p.addUserDebugParameter("blue", 0, 1, 0)
alphaSlider = p.addUserDebugParameter("alpha", 0, 1, 0.5)

while (1):
  red = p.readUserDebugParameter(redSlider)
  green = p.readUserDebugParameter(greenSlider)
  blue = p.readUserDebugParameter(blueSlider)
  alpha = p.readUserDebugParameter(alphaSlider)
  p.changeVisualShape(sphereUid, -1, rgbaColor=[red, green, blue, alpha])
  p.getCameraImage(320,
                   200,
                   flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,
                   renderer=p.ER_BULLET_HARDWARE_OPENGL)
  time.sleep(0.01)
