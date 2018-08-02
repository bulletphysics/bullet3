import pybullet as p
from time import sleep
import matplotlib.pyplot as plt
import numpy as np

physicsClient = p.connect(p.GUI)

p.setGravity(0,0,0)
bearStartPos1 = [-3.3,0,0]
bearStartOrientation1 = p.getQuaternionFromEuler([0,0,0])
bearId1 = p.loadURDF("plane.urdf", bearStartPos1, bearStartOrientation1)
bearStartPos2 = [0,0,0]
bearStartOrientation2 = p.getQuaternionFromEuler([0,0,0])
bearId2 = p.loadURDF("teddy_large.urdf",bearStartPos2, bearStartOrientation2)
textureId = p.loadTexture("checker_grid.jpg")
#p.changeVisualShape(objectUniqueId=0, linkIndex=-1, textureUniqueId=textureId)
#p.changeVisualShape(objectUniqueId=1, linkIndex=-1, textureUniqueId=textureId)


useRealTimeSimulation = 1

if (useRealTimeSimulation):
	p.setRealTimeSimulation(1)

while 1:
	if (useRealTimeSimulation):
		camera = p.getDebugVisualizerCamera()
		viewMat = camera[2]
		projMat = camera[3]
		#An example of setting the view matrix for the projective texture.
		#viewMat = p.computeViewMatrix(cameraEyePosition=[7,0,0], cameraTargetPosition=[0,0,0], cameraUpVector=[0,0,1])
		p.getCameraImage(300, 300, renderer=p.ER_BULLET_HARDWARE_OPENGL, flags=p.ER_USE_PROJECTIVE_TEXTURE, projectiveTextureView=viewMat, projectiveTextureProj=projMat)
		p.setGravity(0,0,0)
	else:
		p.stepSimulation()
