import pybullet as p
p.connect(p.GUI)
r2d2 = p.loadURDF("r2d2.urdf",[0,0,1])
for l in range (p.getNumJoints(r2d2)):
	print(p.getJointInfo(r2d2,l))

p.loadURDF("r2d2.urdf",[2,0,1])
p.loadURDF("r2d2.urdf",[4,0,1])

p.getCameraImage(320,200,flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)
segLinkIndex=1
verbose = 0

while (1):
	keys = p.getKeyboardEvents()
	#for k in keys:
	#	print("key=",k,"state=", keys[k])
	if ord('1') in keys:
		state = keys[ord('1')]
		if (state & p.KEY_WAS_RELEASED):
			verbose = 1-verbose
	if ord('s') in keys:
		state = keys[ord('s')]
		if (state & p.KEY_WAS_RELEASED):
			segLinkIndex = 1-segLinkIndex
			#print("segLinkIndex=",segLinkIndex)
	flags = 0
	if (segLinkIndex):
		flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX
		
	img = p.getCameraImage(320,200,flags=flags)
	#print(img[0],img[1])
	seg=img[4]
	if (verbose):
		for pixel in seg:
			if (pixel>=0):
				obUid = pixel & ((1<<24)-1)
				linkIndex = (pixel >> 24)-1
				print("obUid=",obUid,"linkIndex=",linkIndex)
	
	
	p.stepSimulation()