#See pybullet quickstart guide here:
#https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#

# Create a Tiltbrush-like app, drawing lines using any controller
# Line width can be changed

import pybullet as p
#assume that the VR physics server is already started before
p.connect(p.SHARED_MEMORY)
p.setInternalSimFlags(0)#don't load default robot assets etc
p.resetSimulation()
p.loadURDF("plane.urdf")

prev=[None]*p.VR_MAX_CONTROLLERS
colors=[0.,0.5,0.5]*p.VR_MAX_CONTROLLERS
widths = [3]*p.VR_MAX_CONTROLLERS

#use a few default colors
colors[0] = [0,0,0]
colors[1] = [0.5,0,0]
colors[2] = [0,0.5,0]
colors[3] = [0,0,0.5]
colors[4] = [0.5,0.5,0.]
colors[5] = [.5,.5,.5]

while True:
	events = p.getVREvents()

	for e in (events):
		if (e[6][33]&2):
			prev[e[0]] = e[1]
		if (e[6][32]&2):
			widths[e[0]]=widths[e[0]]+1
			if (widths[e[0]]>20):
				widths[e[0]] = 1
		if (e[6][1]&2):
			p.resetSimulation()
			#p.setGravity(0,0,-10)
			p.removeAllUserDebugItems()
			p.loadURDF("plane.urdf")
		if (e[6][33]==1):
			pt = prev[e[0]]
			
			#print(prev[e[0]])
			#print(e[1])
			diff = [pt[0]-e[1][0],pt[1]-e[1][1],pt[2]-e[1][2]]
			lenSqr =	diff[0]*diff[0]+diff[1]*diff[1]+diff[2]*diff[2]
			ptDistThreshold = 0.01
			if (lenSqr>(ptDistThreshold*ptDistThreshold)):
				p.addUserDebugLine(e[1],prev[e[0]],colors[e[0]],widths[e[0]])
				#p.loadURDF("cube_small.urdf",e[1])
				colors[e[0]] = [1-colors[e[0]][0],1-colors[e[0]][1],1-colors[e[0]][2]]
				prev[e[0]] = e[1]			