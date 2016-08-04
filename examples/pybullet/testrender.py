import numpy as np
import matplotlib.pyplot as plt
import pybullet

pybullet.connect(pybullet.GUI)
pybullet.loadURDF("r2d2.urdf")

camTargetPos = [0,0,0]
#cameraUp = [0,0,1]
cameraPos = [3,3,3]
yaw = 40.0
pitch = 0.0
upAxisIndex = 1
camDistance = 3
pixelWidth = 640
pixelHeight = 480
nearPlane = 0.01
farPlane = 1000

fov = 60

#img_arr = pybullet.renderImage(pixelWidth, pixelHeight)
#renderImage(w, h, view[16], projection[16])
#img_arr = pybullet.renderImage(pixelWidth, pixelHeight, cameraPos, camTargetPos, cameraUp, nearPlane, farPlane)
img_arr = pybullet.renderImage(pixelWidth, pixelHeight, camTargetPos, camDistance, yaw, pitch, upAxisIndex, nearPlane, farPlane, fov)

w=img_arr[0] #width of the image, in pixels
h=img_arr[1] #height of the image, in pixels
rgb=img_arr[2] #color data RGB
dep=img_arr[3] #depth data


# reshape creates np array
np_img_arr = np.reshape(rgb, (pixelHeight, pixelWidth, 4))
np_img_arr = np_img_arr*(1./255.)

#show
plt.imshow(np_img_arr,interpolation='none')
plt.show()
p.resetSimulation()
