import pybullet as p
from time import sleep
from PIL import Image
import matplotlib.pyplot as plt
import numpy as np

physicsClient = p.connect(p.GUI)

p.setGravity(0,0,0)
bearStartPos1 = [-3.3,0,0]
bearStartOrientation1 = p.getQuaternionFromEuler([0,0,0])
bearId1 = p.loadURDF("teddy_large.urdf", bearStartPos1, bearStartOrientation1)
bearStartPos2 = [0,0,0]
bearStartOrientation2 = p.getQuaternionFromEuler([0,0,0])
bearId2 = p.loadURDF("teddy_large.urdf",bearStartPos2, bearStartOrientation2)
textureId = p.loadTexture("checker_grid.jpg")
p.changeVisualShape(objectUniqueId=0, linkIndex=-1, textureUniqueId=textureId)
p.changeVisualShape(objectUniqueId=1, linkIndex=-1, textureUniqueId=textureId)


useRealTimeSimulation = 1

if (useRealTimeSimulation):
	p.setRealTimeSimulation(1)

while 1:
	if (useRealTimeSimulation):
		p.setGravity(0,0,0)
		sleep(0.01) # Time in seconds.
	else:
		p.stepSimulation()
