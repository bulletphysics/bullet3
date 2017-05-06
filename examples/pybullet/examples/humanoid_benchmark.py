import pybullet as p
import time
p.connect(p.GUI)
p.setGravity(0,0,-10)
p.setPhysicsEngineParameter(numSolverIterations=5)
p.setPhysicsEngineParameter(fixedTimeStep=1./240.)
p.setPhysicsEngineParameter(numSubSteps=1)

p.loadMJCF("mjcf/humanoid_symmetric.xml")

#first let the humanoid fall
p.setRealTimeSimulation(1)
time.sleep(3)
p.setRealTimeSimulation(0)

#now do a benchmark
print("Starting benchmark")
logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS,"pybullet_humanoid_timings.json")
for i in range(1000):
	p.stepSimulation()
p.stopStateLogging(logId)

print("ended benchmark")