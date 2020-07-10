#make sure to compile pybullet with PYBULLET_USE_NUMPY enabled
#otherwise use testrender.py (slower but compatible without numpy)
#you can also use GUI mode, for faster OpenGL rendering (instead of TinyRender CPU)

import os
import sys
import time
import itertools
import subprocess
import numpy as np
import pybullet
from multiprocessing import Process
import pybullet_data


camTargetPos = [0, 0, 0]
cameraUp = [0, 0, 1]
cameraPos = [1, 1, 1]

pitch = -10.0
roll = 0
upAxisIndex = 2
camDistance = 4
pixelWidth = 84  # 320
pixelHeight = 84  # 200
nearPlane = 0.01
farPlane = 100
fov = 60

import matplotlib.pyplot as plt


class BulletSim():

  def __init__(self, connection_mode, *argv):
    self.connection_mode = connection_mode
    self.argv = argv

  def __enter__(self):
    print("connecting")
    optionstring = '--width={} --height={}'.format(pixelWidth, pixelHeight)
    optionstring += ' --window_backend=2 --render_device=0'

    print(self.connection_mode, optionstring, *self.argv)
    cid = pybullet.connect(self.connection_mode, options=optionstring, *self.argv)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    if cid < 0:
      raise ValueError
    print("connected")
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)

    pybullet.resetSimulation()
    pybullet.loadURDF("plane.urdf", [0, 0, -1])
    pybullet.loadURDF("r2d2.urdf")
    pybullet.loadURDF("duck_vhacd.urdf")
    pybullet.setGravity(0, 0, -10)

  def __exit__(self, *_, **__):
    pybullet.disconnect()


def test(num_runs=300, shadow=1, log=True, plot=False):
  if log:
    logId = pybullet.startStateLogging(pybullet.STATE_LOGGING_PROFILE_TIMINGS, "renderTimings")

  if plot:
    plt.ion()

    img = np.random.rand(200, 320)
    #img = [tandard_normal((50,100))
    image = plt.imshow(img, interpolation='none', animated=True, label="blah")
    ax = plt.gca()

  times = np.zeros(num_runs)
  yaw_gen = itertools.cycle(range(0, 360, 10))
  for i, yaw in zip(range(num_runs), yaw_gen):
    pybullet.stepSimulation()
    start = time.time()
    viewMatrix = pybullet.computeViewMatrixFromYawPitchRoll(camTargetPos, camDistance, yaw, pitch,
                                                            roll, upAxisIndex)
    aspect = pixelWidth / pixelHeight
    projectionMatrix = pybullet.computeProjectionMatrixFOV(fov, aspect, nearPlane, farPlane)
    img_arr = pybullet.getCameraImage(pixelWidth,
                                      pixelHeight,
                                      viewMatrix,
                                      projectionMatrix,
                                      shadow=shadow,
                                      lightDirection=[1, 1, 1],
                                      renderer=pybullet.ER_BULLET_HARDWARE_OPENGL)
    #renderer=pybullet.ER_TINY_RENDERER)
    stop = time.time()
    duration = (stop - start)
    if (duration):
      fps = 1. / duration
      #print("fps=",fps)
    else:
      fps = 0
      #print("fps=",fps)
    #print("duraction=",duration)
    #print("fps=",fps)
    times[i] = fps

    if plot:
      rgb = img_arr[2]
      image.set_data(rgb)  #np_img_arr)
      ax.plot([0])
      #plt.draw()
      #plt.show()
      plt.pause(0.01)

  mean_time = float(np.mean(times))
  print("mean: {0} for {1} runs".format(mean_time, num_runs))
  print("")
  if log:
    pybullet.stopStateLogging(logId)
  return mean_time


if __name__ == "__main__":

  res = []

  with BulletSim(pybullet.DIRECT):
    print("\nTesting DIRECT")
    mean_time = test(log=False, plot=True)
    res.append(("tiny", mean_time))

  with BulletSim(pybullet.DIRECT):
    plugin_fn = os.path.join(
        pybullet.__file__.split("bullet3")[0],
        "bullet3/build/lib.linux-x86_64-3.5/eglRenderer.cpython-35m-x86_64-linux-gnu.so")
    plugin = pybullet.loadPlugin(plugin_fn, "_tinyRendererPlugin")
    if plugin < 0:
      print("\nPlugin Failed to load!\n")
      sys.exit()

    print("\nTesting DIRECT+OpenGL")
    mean_time = test(log=True)
    res.append(("plugin", mean_time))

  with BulletSim(pybullet.GUI):
    print("\nTesting GUI")
    mean_time = test(log=False)
    res.append(("egl", mean_time))

  print()
  print("rendertest.py")
  print("back nenv fps fps_tot")
  for r in res:
    print(r[0], "\t", 1, round(r[1]), "\t", round(r[1]))
