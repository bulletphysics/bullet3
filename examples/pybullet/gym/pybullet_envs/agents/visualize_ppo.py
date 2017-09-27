
r"""Script to visualize the trained PPO agent.

python -m pybullet_envs.agents.visualize \
--logdir=ppo
--outdir=/tmp/video/

"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import tensorflow as tf

from agents.scripts import visualize


flags = tf.app.flags
FLAGS = tf.app.flags.FLAGS
flags.DEFINE_string("logdir", None,
                    "Directory to the checkpoint of a training run.")
flags.DEFINE_string("outdir", None,
                    "Local directory for storing the monitoring outdir.")
flags.DEFINE_string("checkpoint", None,
                    "Checkpoint name to load; defaults to most recent.")
flags.DEFINE_integer("num_agents", 1,
                     "How many environments to step in parallel.")
flags.DEFINE_integer("num_episodes", 1, "Minimum number of episodes to render.")
flags.DEFINE_boolean(
    "env_processes", False,
    "Step environments in separate processes to circumvent the GIL.")


def main(_):
  visualize.visualize(FLAGS.logdir, FLAGS.outdir, FLAGS.num_agents,
                      FLAGS.num_episodes, FLAGS.checkpoint, FLAGS.env_processes)


if __name__ == "__main__":
  tf.app.run()

