import os 
import sys
import time
import subprocess
import pybullet as p
from pdb import set_trace
import matplotlib.pyplot as plt
import numpy as np
#subprocess.call(["hardening-check", p.__file__])



p.connect(p.DIRECT)
logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "debugTimings")

plugin = True
if plugin:
    plugin_fn = '/home/argusm/lang/bullet3/build/lib.linux-x86_64-3.5/eglRenderer.cpython-35m-x86_64-linux-gnu.so'
    plugin = p.loadPlugin(plugin_fn,"_tinyRendererPlugin")
    if plugin < 0:
        print("\nPlugin Failed to load!\n")
        sys.exit()
    print("plugin =",plugin)

path = '/home/argusm/lang/bullet3/examples/pybullet/gym/pybullet_data/duck_vhacd.urdf'
p.loadURDF(path,globalScaling=12)
#path = '/home/argusm/lang/gym-grasping/gym_grasping/robots/models/kuka_iiwa/kuka_weiss_bolt.sdf'
#p.loadSDF(path)

start = time.time()

camTargetPos = [0,0,0]
upAxisIndex = 2
nearPlane = 0.01
farPlane = 100
camDistance = 2
pixelWidth = 128
pixelHeight = 128
fov = 60

plot = False
anim = True
if plot:
    plt.ion()
if anim:
    import matplotlib.animation as manimation
    FFMpegWriter = manimation.writers['ffmpeg']
    metadata = dict(title='Movie Test', artist='Matplotlib',
                    comment='Movie support!')
    writer = FFMpegWriter(fps=15, metadata=metadata)
if plot or anim:
    fig = plt.figure()
    img = np.random.rand(pixelWidth,pixelHeight)
    image = plt.imshow(img,interpolation='none',animated=True,label="blah")
    ax = plt.gca()
    ax.set_axis_off()
    ax.set_aspect('equal')
    plt.subplots_adjust(wspace=0, hspace=0, left=0, bottom=0, right=1, top=1)


try:
    iter = range(0,360,10)
    with writer.saving(fig, "debug.mp4", len(iter)):
        for i,yaw in enumerate(iter):
            viewMatrix = p.computeViewMatrixFromYawPitchRoll(camTargetPos, camDistance, 0, yaw-90, 0, upAxisIndex)
            aspect = pixelWidth / pixelHeight;
            projectionMatrix = p.computeProjectionMatrixFOV(fov, aspect, nearPlane, farPlane);

            hight, width, img_arr, deept_arr, obj_arr = p.getCameraImage(pixelWidth,pixelHeight,viewMatrix,projectionMatrix)
            if plot:
                image.set_data(img_arr)#np_img_arr)
                ax.plot([0])
                #plt.draw()
                #plt.show()
                plt.pause(0.01)
            if anim:
                image.set_data(img_arr)#np_img_arr)
                ax.plot([0])
                writer.grab_frame()

            if i % 100 == 0 and i > 0:
                print("FPS",100/(time.time()-start))
            start = time.time()
finally:
    p.stopStateLogging(logId)
