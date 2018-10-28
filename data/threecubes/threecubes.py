import pybullet as p
p.connect(p.DIRECT)
p.loadPlugin("eglRendererPlugin")
p.loadSDF("newsdf.sdf")
while (1):
	p.getCameraImage(320,240, flags=p.ER_NO_SEGMENTATION_MASK)
	p.stepSimulation()
	