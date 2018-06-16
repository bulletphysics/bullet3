import pybullet as p
import time
import math

p.connect(p.DIRECT)#GUI)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
p.loadURDF("samurai.urdf")
p.loadURDF("r2d2.urdf",[3,3,1])


rayFrom=[]
rayTo=[]

numRays = 2048
rayLen = 13


for i in range (numRays):
	rayFrom.append([0,0,1])
	rayTo.append([rayLen*math.sin(2.*math.pi*float(i)/numRays), rayLen*math.cos(2.*math.pi*float(i)/numRays),1])

timingLog = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS,"rayCastBench.json")
for i in range (10):
	p.stepSimulation()
	for j in range (8):
		results = p.rayTestBatch(rayFrom,rayTo,j+1)
	
	for i in range (10):
		p.removeAllUserDebugItems()	
		p.removeAllUserDebugItems()	
		
	
	rayHitColor = [1,0,0]
	rayMissColor = [0,1,0]
	#for i in range (numRays):
	#	if (results[i][0]<0):
	#		p.addUserDebugLine(rayFrom[i],rayTo[i], rayMissColor)
	#	else:
	#		p.addUserDebugLine(rayFrom[i],rayTo[i], rayHitColor)
		
	#time.sleep(1./240.)
	
p.stopStateLogging(timingLog)