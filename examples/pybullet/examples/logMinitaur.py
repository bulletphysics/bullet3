import pybullet as p
cid = p.connect(p.SHARED_MEMORY)
if (cid < 0) :
	p.connect(p.GUI)
p.loadURDF("plane.urdf")

quadruped = p.loadURDF("quadruped/quadruped.urdf")
logId = p.startStateLogging(p.STATE_LOGGING_MINITAUR,"LOG00048.TXT",[quadruped])
p.stepSimulation()
p.stepSimulation()
p.stepSimulation()
p.stepSimulation()
p.stepSimulation()

p.stopStateLogging(logId)
