"""An example to run the minitaur environment of standing with four legs.

"""
import numpy as np
import tensorflow as tf
from pybullet_envs.minitaur.envs import minitaur_four_leg_stand_env


FLAGS = tf.flags.FLAGS
tf.flags.DEFINE_string("log_path", None, "The directory to write the log file.")
NUM_LEGS = 4
kroll = 3.0
kpitch = 3.0


def feed_forward_only_control_example(log_path=None):
  """An example of hand-tuned controller for minitaur standing with four legs.

  Args:
    log_path: The directory that the log files are written to. If log_path is
      None, no logs will be written.
  """
  steps = 1000
  episodes = 1

  environment = minitaur_four_leg_stand_env.MinitaurFourLegStandEnv(
      on_rack=False,
      log_path=log_path,
      urdf_version=minitaur_four_leg_stand_env.RAINBOW_DASH_V0_URDF_VERSION,
      remove_default_joint_damping=True,
      hard_reset=True,
      motor_kp=1.0,
      motor_kd=0.015,
      control_latency=0.015,
      pd_latency=0.003,
      control_time_step=0.006,
      action_repeat=6,
      env_randomizer=None,
      render=True)

  np.random.seed(100)
  avg_reward = 0
  for i in range(episodes):
    sum_reward = 0
    observation = environment.reset()
    for _ in range(steps):
      action = [0] * 4
      uroll = kroll * observation[0]
      upitch = kpitch * observation[1]
      action[0] = upitch - uroll
      action[1] = -upitch - uroll
      action[2] = upitch + uroll
      action[3] = -upitch + uroll
      observation, reward, done, _ = environment.step(action)
      sum_reward += reward
      if done:
        break
    tf.logging.info("reward {}: {}".format(i, sum_reward))
    avg_reward += sum_reward
  tf.logging.info("avg_reward: {}\n\n\n".format(avg_reward / episodes))


def main(unused_argv):
  feed_forward_only_control_example(log_path=FLAGS.log_path)


if __name__ == "__main__":
  tf.logging.set_verbosity(tf.logging.INFO)
  tf.app.run()
