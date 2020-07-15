import pybullet as p
import time
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("table/table.urdf", 0.5000000, 0.00000, -.820000, 0.000000, 0.000000, 0.0, 1.0)
p.setGravity(0, 0, -10)
arm = p.loadURDF("widowx/widowx.urdf", useFixedBase=True)

p.resetBasePositionAndOrientation(arm, [-0.098612, -0.000726, -0.194018],
                                  [0.000000, 0.000000, 0.000000, 1.000000])

while (1):
  p.stepSimulation()
  time.sleep(0.01)
  #p.saveWorld("test.py")
  viewMat = p.getDebugVisualizerCamera()[2]
  #projMatrix = [0.7499999403953552, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0000200271606445, -1.0, 0.0, 0.0, -0.02000020071864128, 0.0]
  projMatrix = p.getDebugVisualizerCamera()[3]
  width = 640
  height = 480
  img_arr = p.getCameraImage(width=width,
                             height=height,
                             viewMatrix=viewMat,
                             projectionMatrix=projMatrix)
