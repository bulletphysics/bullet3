import time
import pybullet as p
import pybullet_data as pd
useMaximalCoordinatesA = True
useMaximalCoordinatesB = True
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setAdditionalSearchPath(pd.getDataPath())
cube=p.loadURDF("cube_rotate.urdf",useMaximalCoordinates=useMaximalCoordinatesA)
p.loadURDF("sphere2.urdf",[0,0,2],useMaximalCoordinates=useMaximalCoordinatesB)
p.setGravity(0,0,-10)
p.setJointMotorControl2(cube,0,p.VELOCITY_CONTROL,targetVelocity=1, force=100)
while (1):
  p.stepSimulation()
  time.sleep(1./240.)

