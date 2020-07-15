import pybullet as p
import time
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.resetSimulation()
timinglog = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "loadingBenchVR.json")
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
print("load plane.urdf")
p.loadURDF("plane.urdf")
print("load r2d2.urdf")

p.loadURDF("r2d2.urdf", 0, 0, 1)
print("load kitchen/1.sdf")
p.loadSDF("kitchens/1.sdf")
print("load 100 times plate.urdf")
for i in range(100):
  p.loadURDF("dinnerware/plate.urdf", 0, i, 1)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

p.stopStateLogging(timinglog)
print("stopped state logging")
p.getCameraImage(320, 200)

while (1):
  p.stepSimulation()
