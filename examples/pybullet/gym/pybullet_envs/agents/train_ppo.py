r"""Script to use Proximal Policy Gradient for the minitaur environments.

Run:
 python train_ppo.py --logdif=/tmp/train --config=minitaur_pybullet

"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import datetime
import os
import tensorflow as tf

from agents import tools
from agents.scripts import train
from agents.scripts import utility
from . import config_ppo


flags = tf.app.flags
FLAGS = tf.app.flags.FLAGS

flags.DEFINE_string(
    'logdir', None,
    'Base directory to store logs.')
flags.DEFINE_string(
    'config', None,
    'Configuration to execute.')
flags.DEFINE_string(
    'timestamp', datetime.datetime.now().strftime('%Y%m%dT%H%M%S'),
    'Sub directory to store logs.')


def main(_):
  """Create or load configuration and launch the trainer."""
  config = tools.AttrDict(getattr(config_ppo, FLAGS.config)())
  logdir = FLAGS.logdir and os.path.join(
      FLAGS.logdir, '{}-{}'.format(FLAGS.timestamp, FLAGS.config))
  utility.save_config(config, logdir)
  for score in train.train(config, env_processes=True):
    tf.logging.info(str(score))


if __name__ == '__main__':
  tf.app.run()

