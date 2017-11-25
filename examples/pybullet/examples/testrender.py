
#testrender.py is a bit slower than testrender_np.py: pixels are copied from C to Python one by one


import matplotlib.pyplot as plt 
import pybullet
import time

plt.ion()

img = [[1,2,3]*50]*100#np.random.rand(200, 320)
#img = [tandard_normal((50,100))
image = plt.imshow(img,interpolation='none',animated=True,label="blah")
ax = plt.gca()


pybullet.connect(pybullet.DIRECT)
pybullet.loadURDF("plane.urdf",[0,0,-1])
pybullet.loadURDF("r2d2.urdf")

pybullet.setGravity(0,0,-10)
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
while(1): 
  for yaw in range (0,360,10) :
    pybullet.stepSimulation()
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
    #print(rgb)
    print ('width = %d height = %d' % (w,h))

    #note that sending the data using imshow to matplotlib is really slow, so we use set_data

    #plt.imshow(rgb,interpolation='none')
    image.set_data(rgb)
    ax.plot([0])
    #plt.draw()
    #plt.show()
    plt.pause(0.01)
    
main_stop = time.time()

print ("Total time %f" % (main_stop - main_start))

pybullet.resetSimulation()
