import pybullet as p
import time
import math

useGui = True

if (useGui):
	p.connect(p.GUI)
else:
	p.connect(p.DIRECT)

#p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)

p.loadURDF("samurai.urdf")
p.loadURDF("r2d2.urdf",[3,3,1])


rayFrom=[]
rayTo=[]

numRays = 1024
rayLen = 13


for i in range (numRays):
	rayFrom.append([0,0,1])
	rayTo.append([rayLen*math.sin(2.*math.pi*float(i)/numRays), rayLen*math.cos(2.*math.pi*float(i)/numRays),1])

if (not useGui):
	timingLog = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS,"rayCastBench.json")

numSteps = 10
if (useGui):
	numSteps = 327680

for i in range (numSteps):
	p.stepSimulation()
	for j in range (8):
		results = p.rayTestBatch(rayFrom,rayTo,j+1)
	
	for i in range (10):
		p.removeAllUserDebugItems()	
		
		
	
	rayHitColor = [1,0,0]
	rayMissColor = [0,1,0]
	if (useGui):
		p.removeAllUserDebugItems()	
		for i in range (numRays):
			hitObjectUid=results[i][0]
			
			
			if (hitObjectUid<0):
				p.addUserDebugLine(rayFrom[i],rayTo[i], rayMissColor)
			else:
				hitPosition = results[i][3]
				p.addUserDebugLine(rayFrom[i],hitPosition, rayHitColor)
		
	#time.sleep(1./240.)
	
if (not useGui):
	p.stopStateLogging(timingLog)