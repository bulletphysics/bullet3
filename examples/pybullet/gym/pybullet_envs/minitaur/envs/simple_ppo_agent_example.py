r"""An example to use simple_ppo_agent.

A galloping example:
blaze run -c opt \
//robotics/reinforcement_learning/minitaur/agents:simple_ppo_agent_example -- \
--logdir=/cns/ij-d/home/jietan/experiment/minitaur_vizier_study_ppo/\
minreact_nonexp_nr_01_186515603_186518344/15/ \
--checkpoint=model.ckpt-14000000

A trotting example:
blaze run -c opt \
//robotics/reinforcement_learning/minitaur/agents:simple_ppo_agent_example -- \
--logdir=/cns/ij-d/home/jietan/experiment/minitaur_vizier_study_ppo/\
mintrot_nonexp_rd_01_186515603_186518344/24/ \
--checkpoint=model.ckpt-14000000

"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import time

import tf.compat.v1 as tf
from pybullet_envs.agents import utility
from pybullet_envs.minitaur.agents import simple_ppo_agent

flags = tf.app.flags
FLAGS = tf.app.flags.FLAGS
flags.DEFINE_string("logdir", None, "The directory that contains checkpoint and config.")
flags.DEFINE_string("checkpoint", None, "The checkpoint file path.")
flags.DEFINE_string("log_path", None, "The output path to write log.")


def main(argv):
  del argv  # Unused.
  config = utility.load_config(FLAGS.logdir)
  policy_layers = config.policy_layers
  value_layers = config.value_layers
  env = config.env(render=True, log_path=FLAGS.log_path, env_randomizer=None)
  network = config.network

  with tf.Session() as sess:
    agent = simple_ppo_agent.SimplePPOPolicy(sess,
                                             env,
                                             network,
                                             policy_layers=policy_layers,
                                             value_layers=value_layers,
                                             checkpoint=os.path.join(FLAGS.logdir,
                                                                     FLAGS.checkpoint))

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
    tf.logging.info("reward: {}".format(sum_reward))


if __name__ == "__main__":
  tf.app.run(main)
