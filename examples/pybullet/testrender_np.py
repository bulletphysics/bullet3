
#make sure to compile pybullet with PYBULLET_USE_NUMPY enabled, otherwise use testrender.py (slower but compatible without numpy)

import numpy as np
import matplotlib.pyplot as plt
import pybullet
import time

pybullet.connect(pybullet.DIRECT)
pybullet.loadURDF("r2d2.urdf")

camTargetPos = [0,0,0]
cameraUp = [0,0,1]
cameraPos = [1,1,1]
yaw = 40
pitch = 10.0

roll=0
upAxisIndex = 2
camDistance = 4
pixelWidth = 1920
pixelHeight = 1080
nearPlane = 0.01
farPlane = 1000

fov = 60

main_start = time.time()
for pitch in range (0,360,10) :
    start = time.time()
    viewMatrix = pybullet.computeViewMatrixFromYawPitchRoll(camTargetPos, camDistance, yaw, pitch, roll, upAxisIndex)
    aspect = pixelWidth / pixelHeight;
    projectionMatrix = pybullet.computeProjectionMatrixFOV(fov, aspect, nearPlane, farPlane);
    img_arr = pybullet.getCameraImage(pixelWidth, pixelHeight, viewMatrix,projectionMatrix, [0,1,0])
    stop = time.time()
    print ("renderImage %f" % (stop - start))

    w=img_arr[0] #width of the image, in pixels
    h=img_arr[1] #height of the image, in pixels
    rgb=img_arr[2] #color data RGB
    dep=img_arr[3] #depth data

    print 'width = %d height = %d' % (w,h)

    #note that sending the data to matplotlib is really slow
    #show
    plt.imshow(rgb,interpolation='none')
    #plt.show()
    plt.pause(0.01)

main_stop = time.time()

print ("Total time %f" % (main_stop - main_start))

pybullet.resetSimulation()
