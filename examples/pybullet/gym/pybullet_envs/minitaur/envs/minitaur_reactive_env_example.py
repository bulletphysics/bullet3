r"""An example to use simple_ppo_agent.

blaze run -c opt \
//robotics/reinforcement_learning/minitaur/envs:minitaur_reactive_env_example
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import time
import tensorflow as tf
from agents.scripts import utility
import simple_ppo_agent

flags = tf.app.flags
FLAGS = tf.app.flags.FLAGS
LOG_DIR = (
    "testdata/minitaur_reactive_env_test")
CHECKPOINT = "model.ckpt-14000000"


def main(argv):
  del argv  # Unused.
  config = utility.load_config(LOG_DIR)
  policy_layers = config.policy_layers
  value_layers = config.value_layers
  env = config.env(render=True)
  network = config.network

  with tf.Session() as sess:
    agent = simple_ppo_agent.SimplePPOPolicy(
        sess,
        env,
        network,
        policy_layers=policy_layers,
        value_layers=value_layers,
        checkpoint=os.path.join(LOG_DIR, CHECKPOINT))

    sum_reward = 0
    observation = env.reset()
    while True:
      action = agent.get_action([observation])
      observation, reward, done, _ = env.step(action[0])
      # This sleep is to prevent serial communication error on the real robot.
      time.sleep(0.002)
      sum_reward += reward
      if done:
        break
    tf.logging.info("reward: %s", sum_reward)


if __name__ == "__main__":
  tf.app.run(main)
