import os 
import sys
import time
import subprocess
import pybullet as p
from pdb import set_trace
import matplotlib.pyplot as plt
#subprocess.call(["hardening-check", p.__file__])

p.connect(p.DIRECT)
logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "debugTimings")

plugin_fn = '/home/argusm/lang/bullet3/build/lib.linux-x86_64-3.5/eglRenderer.cpython-35m-x86_64-linux-gnu.so'
plugin = p.loadPlugin(plugin_fn,"_tinyRendererPlugin")
if plugin < 0:
    print("\nPlugin Failed to load!\n")
    sys.exit()

print("plugin =",plugin)

path = '/home/argusm/lang/bullet3/examples/pybullet/gym/pybullet_data/r2d2.urdf'
path = '/home/argusm/lang/gym-grasping/gym_grasping/robots/models/kuka_iiwa/kuka_weiss_bolt.sdf'
#p.loadURDF(path)
p.loadSDF(path)

start = time.time()

plot = False
try:
    for i in range(10):
        hight, width, img_arr, deept_arr, obj_arr = p.getCameraImage(80,80)
        if plot:
            plt.imshow(img_arr[:,:,:3])
            plt.show()
        if i % 100 == 0 and i > 0:
            print("FPS",100/(time.time()-start))
        start = time.time()
finally:
    p.stopStateLogging(logId)



