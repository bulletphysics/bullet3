#!/usr/bin/env python
import os, logging, gym
from baselines import logger
from baselines.common import set_global_seeds
from baselines.common.misc_util import boolean_flag
from baselines import bench
from baselines.a2c.a2c import learn
from baselines.common.vec_env.subproc_vec_env import SubprocVecEnv
from baselines.common.vec_env.vec_frame_stack import VecFrameStack
import time

import gym
from gym import spaces
import pybullet as p
from itertools import cycle
import numpy as np

camTargetPos = [0, 0, 0]
cameraUp = [0, 0, 1]
cameraPos = [1, 1, 1]
pitch = -10.0
roll = 0
upAxisIndex = 2
camDistance = 4
pixelWidth = 320
pixelHeight = 200
nearPlane = 0.01
farPlane = 100
fov = 60


class TestEnv(gym.Env):

  def __init__(
      self,
      renderer='tiny',  # ('tiny', 'egl', 'debug')
  ):
    self.action_space = spaces.Discrete(2)
    self.iter = cycle(range(0, 360, 10))

    # how we want to show
    assert renderer in ('tiny', 'egl', 'debug', 'plugin')
    self._renderer = renderer
    self._render_width = 84
    self._render_height = 84
    # connecting
    if self._renderer == "tiny" or self._renderer == "plugin":
      optionstring = '--width={} --height={}'.format(self._render_width, self._render_height)
      p.connect(p.DIRECT, options=optionstring)

      if self._renderer == "plugin":
        plugin_fn = os.path.join(
            p.__file__.split("bullet3")[0],
            "bullet3/build/lib.linux-x86_64-3.5/eglRenderer.cpython-35m-x86_64-linux-gnu.so")
        plugin = p.loadPlugin(plugin_fn, "_tinyRendererPlugin")
        if plugin < 0:
          print("\nPlugin Failed to load! Try installing via `pip install -e .`\n")
          sys.exit()
        print("plugin =", plugin)

    elif self._renderer == "egl":
      optionstring = '--width={} --height={}'.format(self._render_width, self._render_height)
      optionstring += ' --window_backend=2 --render_device=0'
      p.connect(p.GUI, options=optionstring)

    elif self._renderer == "debug":
      #print("Connection: SHARED_MEMORY")
      #cid = p.connect(p.SHARED_MEMORY)
      #if (cid<0):
      cid = p.connect(p.GUI)
      p.resetDebugVisualizerCamera(1.3, 180, -41, [0.52, -0.2, -0.33])

    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)

  def __del__(self):
    p.disconnect()

  def reset(self):
    pass

  def step(self, action):
    p.stepSimulation()
    start = time.time()
    yaw = next(self.iter)
    viewMatrix = p.computeViewMatrixFromYawPitchRoll(camTargetPos, camDistance, yaw, pitch, roll,
                                                     upAxisIndex)
    aspect = pixelWidth / pixelHeight
    projectionMatrix = p.computeProjectionMatrixFOV(fov, aspect, nearPlane, farPlane)
    img_arr = p.getCameraImage(pixelWidth,
                               pixelHeight,
                               viewMatrix,
                               projectionMatrix,
                               shadow=1,
                               lightDirection=[1, 1, 1],
                               renderer=p.ER_BULLET_HARDWARE_OPENGL)
    #renderer=pybullet.ER_TINY_RENDERER)
    self._observation = img_arr[2]
    return np.array(self._observation), 0, 0, {}

  def seed(self, seed=None):
    pass


def train(env_id, num_timesteps=300, seed=0, num_env=2, renderer='tiny'):

  def make_env(rank):

    def _thunk():
      if env_id == "TestEnv":
        env = TestEnv(renderer=renderer)  #gym.make(env_id)
      else:
        env = gym.make(env_id)
      env.seed(seed + rank)
      env = bench.Monitor(env, logger.get_dir() and os.path.join(logger.get_dir(), str(rank)))
      gym.logger.setLevel(logging.WARN)
      # only clip rewards when not evaluating
      return env

    return _thunk

  set_global_seeds(seed)
  env = SubprocVecEnv([make_env(i) for i in range(num_env)])

  env.reset()
  start = time.time()
  for i in range(num_timesteps):
    action = [env.action_space.sample() for _ in range(num_env)]
    env.step(action)
  stop = time.time()
  duration = (stop - start)
  if (duration):
    fps = num_timesteps / duration
  else:
    fps = 0
  env.close()
  return num_env, fps


if __name__ == "__main__":
  env_id = "TestEnv"
  res = []

  for renderer in ('tiny', 'plugin', 'egl'):
    for i in (1, 8):
      tmp = train(env_id, num_env=i, renderer=renderer)
      print(renderer, tmp)
      res.append((renderer, tmp))
  print()
  print("rendertest_sync.py")
  print("back nenv fps fps_tot")
  for renderer, i in res:
    print(renderer, '\t', i[0], round(i[1]), '\t', round(i[0] * i[1]))
