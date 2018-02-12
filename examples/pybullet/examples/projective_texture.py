import pybullet as p
from time import sleep
from PIL import Image
import matplotlib.pyplot as plt
import numpy as np

physicsClient = p.connect(p.GUI)

p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("cube.urdf",cubeStartPos, cubeStartOrientation)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
textureId = p.loadTexture("checker_blue_test.png")
p.changeVisualShape(objectUniqueId=0, linkIndex=-1, textureUniqueId=textureId)
p.changeVisualShape(objectUniqueId=1, linkIndex=-1, textureUniqueId=textureId)

fov = 70
pixelWidth = 640
pixelHeight = 512
aspect = pixelWidth / pixelHeight;
nearPlane = 0.01
farPlane = 100
projectionMatrix = p.computeProjectionMatrixFOV(fov, aspect, nearPlane, farPlane);
viewMatrixLeft = p.computeViewMatrix(cameraEyePosition=[0.045, 3.0, 3.0], cameraTargetPosition=[0.045,0,0], cameraUpVector=[0,0,1])
viewMatrixRight = p.computeViewMatrix(cameraEyePosition=[-0.045, 3.0, 3.0], cameraTargetPosition=[-0.045,0,0], cameraUpVector=[0,0,1])
leftImage = p.getCameraImage(width=640,height=512,viewMatrix=viewMatrixLeft,projectionMatrix=projectionMatrix,renderer=p.ER_BULLET_HARDWARE_OPENGL)
rightImage = p.getCameraImage(width=640,height=512,viewMatrix=viewMatrixRight,projectionMatrix=projectionMatrix,renderer=p.ER_BULLET_HARDWARE_OPENGL)

left_img = np.reshape(np.asarray(leftImage[2], dtype=np.float32), (512, 640, 4))
left_img /= 255
plt.imsave("left_image.png", left_img)
right_img = np.reshape(np.asarray(rightImage[2], dtype=np.float32), (512, 640, 4))
right_img /= 255
plt.imsave("right_image.png", right_img)

useRealTimeSimulation = 1

if (useRealTimeSimulation):
	p.setRealTimeSimulation(1)

while 1:
	if (useRealTimeSimulation):
		p.setGravity(0,0,-10)
		sleep(0.01) # Time in seconds.
	else:
		p.stepSimulation()
