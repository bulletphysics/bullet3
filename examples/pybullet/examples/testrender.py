
#testrender.py is a bit slower than testrender_np.py: pixels are copied from C to Python one by one


import matplotlib.pyplot as plt 
import pybullet
import time

pybullet.connect(pybullet.DIRECT)
pybullet.loadURDF("plane.urdf",[0,0,-1])
pybullet.loadURDF("r2d2.urdf")

camTargetPos = [0,0,0]
cameraUp = [0,0,1]
cameraPos = [1,1,1]

pitch = -10.0

roll=0
upAxisIndex = 2
camDistance = 4
pixelWidth = 320
pixelHeight = 200
nearPlane = 0.01
farPlane = 100

fov = 60

main_start = time.time()
for yaw in range (0,360,10) :
    start = time.time()
    viewMatrix = pybullet.computeViewMatrixFromYawPitchRoll(camTargetPos, camDistance, yaw, pitch, roll, upAxisIndex)
    aspect = pixelWidth / pixelHeight;
    projectionMatrix = pybullet.computeProjectionMatrixFOV(fov, aspect, nearPlane, farPlane);
    img_arr = pybullet.getCameraImage(pixelWidth, pixelHeight, viewMatrix,projectionMatrix, shadow=1,lightDirection=[1,1,1],renderer=pybullet.ER_BULLET_HARDWARE_OPENGL)
    stop = time.time()
    print ("renderImage %f" % (stop - start))

    w=img_arr[0] #width of the image, in pixels
    h=img_arr[1] #height of the image, in pixels
    rgb=img_arr[2] #color data RGB
    dep=img_arr[3] #depth data

    print ('width = %d height = %d' % (w,h))

    #note that sending the data to matplotlib is really slow

    plt.imshow(rgb,interpolation='none')
    plt.show(block=False)
    plt.pause(0.001)

main_stop = time.time()

print ("Total time %f" % (main_stop - main_start))

pybullet.resetSimulation()
