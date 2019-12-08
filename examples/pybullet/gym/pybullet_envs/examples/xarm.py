import pybullet as p
import pybullet_data as pd
import time
p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())


useFixedBase = True
flags = p.URDF_INITIALIZE_SAT_FEATURES#0#p.URDF_USE_SELF_COLLISION
xarm = p.loadURDF("xarm/xarm6_with_gripper.urdf", flags = flags, useFixedBase=useFixedBase)

while (1):
	p.stepSimulation()
	time.sleep(1./240.)
	
