import pybullet as p
import pybullet_data as pd
import time
p.connect(p.GUI)#, options="--background_color_red=1.0 --background_color_blue=1.0 --background_color_green=1.0")
p.setAdditionalSearchPath(pd.getDataPath())


useFixedBase = True
flags = p.URDF_INITIALIZE_SAT_FEATURES#0#p.URDF_USE_SELF_COLLISION

#plane_pos = [0,0,0]
#plane = p.loadURDF("plane.urdf", plane_pos, flags = flags, useFixedBase=useFixedBase)
table_pos = [0,0,-0.625]
table = p.loadURDF("table/table.urdf", table_pos, flags = flags, useFixedBase=useFixedBase)
xarm = p.loadURDF("xarm/xarm6_robot.urdf", flags = flags, useFixedBase=useFixedBase)

jointIds = []
paramIds = []

for j in range(p.getNumJoints(xarm)):
  p.changeDynamics(xarm, j, linearDamping=0, angularDamping=0)
  info = p.getJointInfo(xarm, j)
  #print(info)
  jointName = info[1]
  jointType = info[2]
  if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
    jointIds.append(j)
    paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), -4, 4, 0))
  
skip_cam_frames = 10  

while (1):
	p.stepSimulation()
	for i in range(len(paramIds)):
		c = paramIds[i]
		targetPos = p.readUserDebugParameter(c)
		p.setJointMotorControl2(xarm, jointIds[i], p.POSITION_CONTROL, targetPos, force=5 * 240.)
	skip_cam_frames -= 1
	if (skip_cam_frames<0):
		p.getCameraImage(320,200, renderer=p.ER_BULLET_HARDWARE_OPENGL )
		skip_cam_frames = 10
	time.sleep(1./240.)
	
