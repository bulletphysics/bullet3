"""An example to run the minitaur environment of trotting gait.

"""
import time
import os
import numpy as np
import tensorflow as tf
from pybullet_envs.minitaur.envs import minitaur_gym_env
from pybullet_envs.minitaur.envs import minitaur_trotting_env

#FLAGS = tf.flags.FLAGS
#tf.flags.DEFINE_string("log_path", None, "The directory to write the log file.")


def main(unused_argv):
  environment = minitaur_trotting_env.MinitaurTrottingEnv(
      urdf_version=minitaur_gym_env.RAINBOW_DASH_V0_URDF_VERSION,
      use_signal_in_observation=False,
      use_angle_in_observation=False,
      render=True,
      log_path=os.getcwd())

  np.random.seed(100)
  sum_reward = 0
  environment.reset()

  steps = 5000
  for _ in range(steps):
    # Sleep to prevent serial buffer overflow on microcontroller.
    time.sleep(0.002)
    action = [0] * 8
    _, reward, done, _ = environment.step(action)
    sum_reward += reward
    if done:
      break
  tf.logging.info("reward: {}".format(sum_reward))


if __name__ == "__main__":
  tf.logging.set_verbosity(tf.logging.INFO)
  tf.app.run()
