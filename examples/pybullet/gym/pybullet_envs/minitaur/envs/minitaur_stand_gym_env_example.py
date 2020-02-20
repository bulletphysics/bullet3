"""An example to run of the minitaur gym environment with standing up goal.

"""

import math

import numpy as np
import tf.compat.v1 as tf

from pybullet_envs.minitaur.envs import minitaur_stand_gym_env


def StandUpExample():
  """An example that the minitaur stands up."""
  steps = 1000
  environment = minitaur_stand_gym_env.MinitaurStandGymEnv(render=True,
                                                           motor_velocity_limit=np.inf)
  action = [0.5]
  _, _, done, _ = environment.step(action)
  for t in range(steps):
    # A policy that oscillates between -1 and 1
    action = [math.sin(t * math.pi * 0.01)]
    _, _, done, _ = environment.step(action)
    if done:
      break


def main(unused_argv):
  StandUpExample()


if __name__ == "__main__":
  tf.logging.set_verbosity(tf.logging.INFO)
  main("unused")
