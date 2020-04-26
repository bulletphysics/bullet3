"""An example to run the gym environment that a minitaur follows a ball.

"""

import math

import tf.compat.v1 as tf
import minitaur_ball_gym_env


def FollowBallManualPolicy():
  """An example of a minitaur following a ball."""
  env = minitaur_ball_gym_env.MinitaurBallGymEnv(render=True,
                                                 pd_control_enabled=True,
                                                 on_rack=False)
  observation = env.reset()
  sum_reward = 0
  steps = 100000
  for _ in range(steps):
    action = [math.tanh(observation[0] * 4)]
    observation, reward, done, _ = env.step(action)
    sum_reward += reward
    if done:
      tf.logging.info("Return is {}".format(sum_reward))
      observation = env.reset()
      sum_reward = 0


def main():
  FollowBallManualPolicy()


if __name__ == '__main__':
  main()
