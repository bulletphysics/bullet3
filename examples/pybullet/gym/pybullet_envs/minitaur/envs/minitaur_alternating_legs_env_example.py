"""An example to run the minitaur environment of alternating legs.

"""
import time

import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import os
import numpy as np
import tf.compat.v1 as tf
from pybullet_envs.minitaur.envs import minitaur_alternating_legs_env
from pybullet_envs.minitaur.envs import minitaur_gym_env
from pybullet_envs.minitaur.envs.env_randomizers import minitaur_alternating_legs_env_randomizer as randomizer_lib

#FLAGS = flags.FLAGS
#flags.DEFINE_string("log_path", None, "The directory to write the log file.")


def hand_tuned_agent(observation, timestamp):
  """A hand tuned controller structure with vizier optimized parameters.

  Args:
    observation: The observation of the environment. It includes the roll, pith
      the speed of roll and pitch changes.
    timestamp: The simulated time since the simulation reset.
  Returns:
    Delta desired motor angles to be added to the reference motion of
      alternating legs for balance.
  """
  roll = observation[0]
  pitch = observation[1]
  roll_dot = observation[2]
  pitch_dot = observation[3]

  # The following gains are hand-tuned. These gains are
  # designed according to traditional robotics techniques. These are linear
  # feedback balance conroller. The idea is that when the base is tilting,
  # the legs in the air should swing more towards the falling direction to catch
  # up falling. At the same time, the legs in the air should extend more to
  # touch ground earlier.
  roll_gain = 1.0
  pitch_gain = 1.0
  roll_dot_gain = 0.1
  pitch_dot_gain = 0.1

  roll_compensation = roll_gain * roll + roll_dot_gain * roll_dot
  pitch_compensation = pitch_gain * pitch + pitch_dot_gain * pitch_dot

  first_leg = [
      0, -pitch_compensation, -pitch_compensation, 0, 0, -pitch_compensation - roll_compensation,
      pitch_compensation + roll_compensation, 0
  ]
  second_leg = [
      -pitch_compensation, 0, 0, -pitch_compensation, pitch_compensation - roll_compensation, 0, 0,
      -pitch_compensation + roll_compensation
  ]
  if (timestamp // minitaur_alternating_legs_env.STEP_PERIOD) % 2:
    return second_leg
  else:
    return first_leg


def hand_tuned_balance_example(log_path=None):
  """An example that the minitaur balances while alternating its legs.

  Args:
    log_path: The directory that the log files are written to. If log_path is
      None, no logs will be written.
  """
  steps = 1000
  episodes = 5

  randomizer = randomizer_lib.MinitaurAlternatingLegsEnvRandomizer()
  environment = minitaur_alternating_legs_env.MinitaurAlternatingLegsEnv(
      urdf_version=minitaur_gym_env.DERPY_V0_URDF_VERSION,
      render=True,
      num_steps_to_log=steps,
      pd_latency=0.002,
      control_latency=0.02,
      remove_default_joint_damping=True,
      on_rack=False,
      env_randomizer=randomizer,
      log_path=log_path)
  np.random.seed(100)
  avg_reward = 0
  for i in range(episodes):
    sum_reward = 0
    observation = environment.reset()
    for _ in range(steps):
      # Sleep to prevent serial buffer overflow on microcontroller.
      time.sleep(0.002)
      action = hand_tuned_agent(observation, environment.minitaur.GetTimeSinceReset())
      observation, reward, done, _ = environment.step(action)
      sum_reward += reward
      if done:
        break
    tf.logging.info("reward {}: {}".format(i, sum_reward))
    avg_reward += sum_reward
  tf.logging.info("avg_reward: {}\n\n\n".format(avg_reward / episodes))


def main(unused_argv):
  hand_tuned_balance_example(log_path=os.getcwd())


if __name__ == "__main__":
  tf.logging.set_verbosity(tf.logging.INFO)
  tf.app.run()
