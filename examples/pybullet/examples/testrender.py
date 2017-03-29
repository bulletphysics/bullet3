import numpy as np
import matplotlib.pyplot as plt
import pybullet

pybullet.connect(pybullet.GUI)
pybullet.loadURDF("r2d2.urdf")

camTargetPos = [0.,0.,0.]
cameraUp = [0,0,1]
cameraPos = [1,1,1]
yaw = 40
pitch = 10.0

roll=0
upAxisIndex = 2
camDistance = 4
pixelWidth = 320
pixelHeight = 240
nearPlane = 0.01
farPlane = 1000
lightDirection = [0,1,0]
lightColor = [1,1,1]#optional argument
fov = 60

#img_arr = pybullet.renderImage(pixelWidth, pixelHeight)
#renderImage(w, h, view[16], projection[16])
#img_arr = pybullet.renderImage(pixelWidth, pixelHeight, cameraPos, camTargetPos, cameraUp, nearPlane, farPlane)
for pitch in range (0,360,10) :
    viewMatrix = pybullet.computeViewMatrixFromYawPitchRoll(camTargetPos, camDistance, yaw, pitch, roll, upAxisIndex)
    aspect = pixelWidth / pixelHeight;
    projectionMatrix = pybullet.computeProjectionMatrixFOV(fov, aspect, nearPlane, farPlane);
    #getCameraImage can also use renderer=pybullet.ER_BULLET_HARDWARE_OPENGL
    img_arr = pybullet.getCameraImage(pixelWidth, pixelHeight, viewMatrix,projectionMatrix, lightDirection,lightColor,renderer=pybullet.ER_TINY_RENDERER)
    w=img_arr[0]
    h=img_arr[1]
    rgb=img_arr[2]
    dep=img_arr[3]
    #print 'width = %d height = %d' % (w,h)
    # reshape creates np array
    np_img_arr = np.reshape(rgb, (h, w, 4))
    np_img_arr = np_img_arr*(1./255.)
    #show
    plt.imshow(np_img_arr,interpolation='none')
    
    plt.pause(0.01)

pybullet.resetSimulation()
