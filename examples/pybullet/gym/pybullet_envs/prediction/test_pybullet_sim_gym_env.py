r"""An example to run of the minitaur gym environment with sine gaits.
"""

import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import math
import numpy as np
from pybullet_envs.prediction import pybullet_sim_gym_env
import argparse
import time

from pybullet_envs.prediction import boxstack_pybullet_sim
from gym.wrappers.monitoring import video_recorder


def ResetPoseExample(steps):
  """An example that the minitaur stands still using the reset pose."""

  environment = pybullet_sim_gym_env.PyBulletSimGymEnv(pybullet_sim_factory=boxstack_pybullet_sim,
                                                       debug_visualization=False,
                                                       render=True,
                                                       action_repeat=30)
  action = [math.pi / 2] * 8

  vid = video_recorder.VideoRecorder(env=environment, path="vid.mp4")

  for _ in range(steps):
    print(_)
    startsim = time.time()
    _, _, done, _ = environment.step(action)
    stopsim = time.time()
    startrender = time.time()
    #environment.render(mode='rgb_array')
    vid.capture_frame()
    stoprender = time.time()
    print("env.step ", (stopsim - startsim))
    print("env.render ", (stoprender - startrender))
    if done:
      environment.reset()


def main():
  parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('--env', help='environment ID (0==reset)', type=int, default=0)
  args = parser.parse_args()
  print("--env=" + str(args.env))

  if (args.env == 0):
    ResetPoseExample(steps=1000)


if __name__ == '__main__':
  main()
