import pybullet as p
import pybullet_data


cid = p.connect(p.SHARED_MEMORY)
if (cid < 0):
  p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

quadruped = p.loadURDF("quadruped/quadruped.urdf")
logId = p.startStateLogging(p.STATE_LOGGING_MINITAUR, "LOG00048.TXT", [quadruped])
p.stepSimulation()
p.stepSimulation()
p.stepSimulation()
p.stepSimulation()
p.stepSimulation()

p.stopStateLogging(logId)
