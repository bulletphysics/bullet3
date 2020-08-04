r"""Running a pre-trained ppo agent on minitaur_trotting_env."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import time
import tf.compat.v1 as tf
from pybullet_envs.minitaur.agents.scripts import utility
import pybullet_data
from pybullet_envs.minitaur.envs import simple_ppo_agent

flags = tf.app.flags
FLAGS = tf.app.flags.FLAGS
LOG_DIR = os.path.join(pybullet_data.getDataPath(), "policies/ppo/minitaur_trotting_env")
CHECKPOINT = "model.ckpt-14000000"


def main(argv):
  del argv  # Unused.
  config = utility.load_config(LOG_DIR)
  policy_layers = config.policy_layers
  value_layers = config.value_layers
  env = config.env(render=True)
  network = config.network

  with tf.Session() as sess:
    agent = simple_ppo_agent.SimplePPOPolicy(sess,
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
      time.sleep(0.002)
      sum_reward += reward
      if done:
        break
    tf.logging.info("reward: %s", sum_reward)


if __name__ == "__main__":
  tf.app.run(main)
