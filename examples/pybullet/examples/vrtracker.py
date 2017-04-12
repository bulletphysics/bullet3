# See pybullet quickstart guide here:
# https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#
# Create a Tiltbrush-like app, drawing lines using any controller
# Line width can be changed

import pybullet as p

CONTROLLER_ID = 0
POSITION=1
ORIENTATION=2
BUTTONS=6

#assume that the VR physics server is already started before
c = p.connect(p.SHARED_MEMORY)
print(c)
if (c<0):
		p.connect(p.GUI)

p.setInternalSimFlags(0)#don't load default robot assets etc
p.resetSimulation()
p.loadURDF("plane.urdf")
p.loadURDF("cube.urdf",0,0,1)
p.setGravity(0,0,-10)
p.setRealTimeSimulation(1)

prevPosition=[None]*p.VR_MAX_CONTROLLERS
colors=[0.,0.5,0.5]*p.VR_MAX_CONTROLLERS
widths = [3]*p.VR_MAX_CONTROLLERS
a=[0,0,0]
#use a few default colors
colors[0] = [0,0,0]
colors[1] = [0.5,0,0]
colors[2] = [0,0.5,0]
colors[3] = [0,0,0.5]
colors[4] = [0.5,0.5,0.]
colors[5] = [.5,.5,.5]

p.startStateLogging(p.STATE_LOGGING_VR_CONTROLLERS, "vr_hmd.bin",deviceTypeFilter=p.VR_DEVICE_HMD)
p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT, "generic_data.bin")
p.startStateLogging(p.STATE_LOGGING_CONTACT_POINTS, "contact_points.bin")

while True:
	events = p.getVREvents(p.VR_DEVICE_HMD+p.VR_DEVICE_GENERIC_TRACKER)
	for e in (events):
		pos = e[POSITION]
		mat = p.getMatrixFromQuaternion(e[ORIENTATION])
		dir0 = [mat[0],mat[3],mat[6]]
		dir1 = [mat[1],mat[4],mat[7]]
		dir2 = [mat[2],mat[5],mat[8]]
		lineLen = 0.1
		dir = [-mat[2],-mat[5],-mat[8]]

		to = [pos[0]+lineLen*dir[0],pos[1]+lineLen*dir[1],pos[2]+lineLen*dir[2]]
		toX = [pos[0]+lineLen*dir0[0],pos[1]+lineLen*dir0[1],pos[2]+lineLen*dir0[2]]
		toY = [pos[0]+lineLen*dir1[0],pos[1]+lineLen*dir1[1],pos[2]+lineLen*dir1[2]]
		toZ = [pos[0]+lineLen*dir2[0],pos[1]+lineLen*dir2[1],pos[2]+lineLen*dir2[2]]
		p.addUserDebugLine(pos,toX,[1,0,0],1)
		p.addUserDebugLine(pos,toY,[0,1,0],1)
		p.addUserDebugLine(pos,toZ,[0,0,1],1)
	
		p.addUserDebugLine(pos,to,[0.5,0.5,0.],1,3)
		
	events = p.getVREvents()

	for e in (events):
		if (e[BUTTONS][33]&p.VR_BUTTON_WAS_TRIGGERED):
			prevPosition[e[CONTROLLER_ID]] = e[POSITION]
		if (e[BUTTONS][32]&p.VR_BUTTON_WAS_TRIGGERED):
			widths[e[CONTROLLER_ID]]=widths[e[0]]+1
			if (widths[e[CONTROLLER_ID]]>20):
				widths[e[CONTROLLER_ID]] = 1
		if (e[BUTTONS][1]&p.VR_BUTTON_WAS_TRIGGERED):
			p.resetSimulation()
			#p.setGravity(0,0,-10)
			p.removeAllUserDebugItems()
			p.loadURDF("plane.urdf")
		if (e[BUTTONS][33]==p.VR_BUTTON_IS_DOWN):
			pt = prevPosition[e[CONTROLLER_ID]]
			
			#print(prevPosition[e[0]])
			#print(e[1])
			diff = [pt[0]-e[POSITION][0],pt[1]-e[POSITION][1],pt[2]-e[POSITION][2]]
			lenSqr =	diff[0]*diff[0]+diff[1]*diff[1]+diff[2]*diff[2]
			ptDistThreshold = 0.01
			if (lenSqr>(ptDistThreshold*ptDistThreshold)):
				p.addUserDebugLine(e[POSITION],prevPosition[e[CONTROLLER_ID]],colors[e[CONTROLLER_ID]],widths[e[CONTROLLER_ID]])
				#p.loadURDF("cube_small.urdf",e[1])
				colors[e[CONTROLLER_ID]] = [1-colors[e[CONTROLLER_ID]][0],1-colors[e[CONTROLLER_ID]][1],1-colors[e[CONTROLLER_ID]][2]]
				prevPosition[e[CONTROLLER_ID]] = e[POSITION]