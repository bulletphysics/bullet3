import pybullet as p
import pybullet_data as pd
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())

plane = p.loadURDF("plane_transparent.urdf")
startPos = [0,0,0.25]
humanoid = p.loadURDF("microtaur.urdf", startPos, useFixedBase=False)
ob = humanoid
jointPositions=[ 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 1.557895, 0.000000, -0.715790, -1.347363, -1.389474, 0.000000, -0.757895, -1.347364, 0.000000, 0.000000, 0.000000, 0.000000, 1.515790, 0.000000, -0.757895, -1.347364, 1.431579, 0.000000, -0.800000, -1.515786 ]
for jointIndex in range (p.getNumJoints(ob)):
	p.resetJointState(ob,jointIndex,jointPositions[jointIndex])
	

gravId = p.addUserDebugParameter("gravity", -10, 10, -10)
jointIds = []
paramIds = []

p.setPhysicsEngineParameter(numSolverIterations=10)
p.changeDynamics(humanoid, -1, linearDamping=0, angularDamping=0)

for j in range(p.getNumJoints(humanoid)):
  p.changeDynamics(humanoid, j, linearDamping=0, angularDamping=0)
  info = p.getJointInfo(humanoid, j)
  #print(info)
  jointName = info[1]
  jointType = info[2]
  if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
    jointIds.append(j)
    paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), -4, 4, jointPositions[j]))

p.setRealTimeSimulation(1)
while (1):
  p.setGravity(0, 0, p.readUserDebugParameter(gravId))
  for i in range(len(paramIds)):
    c = paramIds[i]
    targetPos = p.readUserDebugParameter(c)
    p.setJointMotorControl2(humanoid, jointIds[i], p.POSITION_CONTROL, targetPos, force=5 * 240.)
  p.saveWorld("latest.py")
  time.sleep(0.01)
