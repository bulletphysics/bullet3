import os 
import time
import subprocess
import pybullet as p
#subprocess.call(["hardening-check", p.__file__])

p.connect(p.DIRECT)

plugin_fn = '/home/argusm/lang/bullet3/build/lib.linux-x86_64-3.5/eglRenderer.cpython-35m-x86_64-linux-gnu.so'
plugin = p.loadPlugin(plugin_fn,"_tinyRendererPlugin")
print("plugin =",plugin)

path = '/home/argusm/lang/bullet3/examples/pybullet/gym/pybullet_data/r2d2.urdf'
path = '/home/argusm/lang/gym-grasping/gym_grasping/robots/models/kuka_iiwa/kuka_weiss_bolt.sdf'
#p.loadURDF(path)
p.loadSDF(path)

start = time.time()
for i in range(2000):
    p.getCameraImage(80,80)
    if i % 100 == 0 and i > 0:
        print("FPS",100/(time.time()-start))
        start = time.time()


