import pybullet as p
import time
import pybullet_data

cid = p.connect(p.SHARED_MEMORY)
if (cid < 0):
  p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
q = p.loadURDF("quadruped/quadruped.urdf", useFixedBase=True)
rollId = p.addUserDebugParameter("roll", -1.5, 1.5, 0)
pitchId = p.addUserDebugParameter("pitch", -1.5, 1.5, 0)
yawId = p.addUserDebugParameter("yaw", -1.5, 1.5, 0)
fwdxId = p.addUserDebugParameter("fwd_x", -1, 1, 0)
fwdyId = p.addUserDebugParameter("fwd_y", -1, 1, 0)
fwdzId = p.addUserDebugParameter("fwd_z", -1, 1, 0)

while True:
  roll = p.readUserDebugParameter(rollId)
  pitch = p.readUserDebugParameter(pitchId)
  yaw = p.readUserDebugParameter(yawId)
  x = p.readUserDebugParameter(fwdxId)
  y = p.readUserDebugParameter(fwdyId)
  z = p.readUserDebugParameter(fwdzId)

  orn = p.getQuaternionFromEuler([roll, pitch, yaw])
  p.resetBasePositionAndOrientation(q, [x, y, z], orn)
  #p.stepSimulation()#not really necessary for this demo, no physics used
