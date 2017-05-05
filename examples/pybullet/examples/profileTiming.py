import pybullet as p
import time

p.connect(p.GUI)

t = time.time()+0.1

logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "haha")
while (time.time()<t):
	p.submitProfileTiming("pythontest")
p.stopStateLogging(logId)
		
