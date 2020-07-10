import pybullet as p
import time
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
#p.setPhysicsEngineParameter(constraintSolverType=p.CONSTRAINT_SOLVER_LCP_PGS, globalCFM = 0.0001)
p.setPhysicsEngineParameter(constraintSolverType=p.CONSTRAINT_SOLVER_LCP_DANTZIG,
                            globalCFM=0.000001)
#p.setPhysicsEngineParameter(constraintSolverType=p.CONSTRAINT_SOLVER_LCP_PGS, globalCFM = 0.0001)

p.loadURDF("plane.urdf")
radius = 0.025
distance = 1.86
yaw = 135
pitch = -11
targetPos = [0, 0, 0]

p.setPhysicsEngineParameter(solverResidualThreshold=0.001, numSolverIterations=200)
p.resetDebugVisualizerCamera(distance, yaw, pitch, targetPos)
objectId = -1

for i in range(10):
  objectId = p.loadURDF("cube_small.urdf", [1, 1, radius + i * 2 * radius])

p.changeDynamics(objectId, -1, 100)

timeStep = 1. / 240.
p.setGravity(0, 0, -10)
while (p.isConnected()):
  p.stepSimulation()
  time.sleep(timeStep)
