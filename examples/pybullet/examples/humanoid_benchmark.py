import pybullet as p
import time
import pybullet_data

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
p.setPhysicsEngineParameter(numSolverIterations=5)
p.setPhysicsEngineParameter(fixedTimeStep=1. / 240.)
p.setPhysicsEngineParameter(numSubSteps=1)

p.loadURDF("plane.urdf")

objects = p.loadMJCF("mjcf/humanoid_symmetric.xml")
ob = objects[0]
p.resetBasePositionAndOrientation(ob, [0.789351, 0.962124, 0.113124],
                                  [0.710965, 0.218117, 0.519402, -0.420923])
jointPositions = [
    -0.200226, 0.123925, 0.000000, -0.224016, 0.000000, -0.022247, 0.099119, -0.041829, 0.000000,
    -0.344372, 0.000000, 0.000000, 0.090687, -0.578698, 0.044461, 0.000000, -0.185004, 0.000000,
    0.000000, 0.039517, -0.131217, 0.000000, 0.083382, 0.000000, -0.165303, -0.140802, 0.000000,
    -0.007374, 0.000000
]
for jointIndex in range(p.getNumJoints(ob)):
  p.resetJointState(ob, jointIndex, jointPositions[jointIndex])

#first let the humanoid fall
#p.setRealTimeSimulation(1)
#time.sleep(5)
p.setRealTimeSimulation(0)
#p.saveWorld("lyiing.py")

#now do a benchmark
print("Starting benchmark")
fileName = "pybullet_humanoid_timings.json"

logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, fileName)
for i in range(1000):
  p.stepSimulation()
p.stopStateLogging(logId)

print("ended benchmark")
print("Use Chrome browser, visit about://tracing, and load the %s file" % fileName)
