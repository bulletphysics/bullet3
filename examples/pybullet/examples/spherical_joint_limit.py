import pybullet as p
import pybullet_data as pd

#see spherical_joint_limit.urdf
#lower is the swing range in the joint local X axis
#upper is the swing range in the joint local Y axis
#twist is the twist range rotation around the joint local Z axis
#effort is the maximum force impulse to enforce the joint limit
#<limit effort="1000.0" lower="0.2" upper=".8" twist=".3"/>

import time
dt = 1./240.
p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())

p.setTimeStep(dt)

humanoid = p.loadURDF("spherical_joint_limit.urdf",[0,0,2], useFixedBase=True)

gravxId = p.addUserDebugParameter("grav_x",-20,20,0.3)
gravyId = p.addUserDebugParameter("grav_y",-20,20,0.3)
gravzId = p.addUserDebugParameter("grav_y",-20,20,-10)

index= 0
spherical_index = -1

for j in range(p.getNumJoints(humanoid)):
  if index<7:
    ji = p.getJointInfo(humanoid, j)
    jointType = ji[2]
    if (jointType == p.JOINT_SPHERICAL):
      index+=4
      p.resetJointStateMultiDof(humanoid, j, targetValue=[0,0,0,1], targetVelocity=[0,0,0])
      #p.changeDynamics(humanoid,j,angularDamping=0, linearDamping=0)
      spherical_index = j
      p.setJointMotorControlMultiDof(humanoid, j, controlMode=p.POSITION_CONTROL, 
          targetPosition=[0,0,0,1],  positionGain=0.2,
          targetVelocity=[0,0,0], velocityGain=0,
          force=[0,0,0])
                                   
    if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
      index+=1
      p.resetJointStateMultiDof(humanoid, j, targetValue=[0], targetVelocity=[0])
      p.setJointMotorControlMultiDof(humanoid, j, controlMode=p.POSITION_CONTROL, 
          targetPosition=[0], targetVelocity=[0], force=[0])

p.loadURDF("plane.urdf")

p.setRealTimeSimulation(1)
while p.isConnected():
  gravX = p.readUserDebugParameter(gravxId)
  gravY = p.readUserDebugParameter(gravyId)
  gravZ = p.readUserDebugParameter(gravzId)
  p.setGravity(gravX,gravY,gravZ)
  #p.stepSimulation()
  time.sleep(dt/10.)
