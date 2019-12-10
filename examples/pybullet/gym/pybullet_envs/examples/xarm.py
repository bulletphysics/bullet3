import pybullet as p
import pybullet_data as pd
import time
p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())


useFixedBase = True
flags = p.URDF_INITIALIZE_SAT_FEATURES#0#p.URDF_USE_SELF_COLLISION

#plane_pos = [0,0,0]
#plane = p.loadURDF("plane.urdf", plane_pos, flags = flags, useFixedBase=useFixedBase)
table_pos = [0,0,-0.625]
table = p.loadURDF("table/table.urdf", table_pos, flags = flags, useFixedBase=useFixedBase)
xarm = p.loadURDF("xarm/xarm6_robot.urdf", flags = flags, useFixedBase=useFixedBase)

while (1):
	p.stepSimulation()
	time.sleep(1./240.)
	
