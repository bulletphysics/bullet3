import pybullet as p
import time

p.connect(p.GUI)
logId = p.startStateLogging(p.STATE_LOGGING_ALL_COMMANDS,"commandLog.bin")
p.loadURDF("plane.urdf")
p.loadURDF("r2d2.urdf",[0,0,1])

p.stopStateLogging(logId)
p.resetSimulation();

logId = p.startStateLogging(p.STATE_REPLAY_ALL_COMMANDS,"commandLog.bin")
while(p.isConnected()):
	time.sleep(1./240.)

