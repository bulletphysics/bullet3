"""

blaze build -c opt //experimental/users/jietan/ARS:train_ars
blaze-bin/experimental/users/jietan/ARS/train_ars \
--logdir=/cns/ij-d/home/jietan/experiment/ARS/test1 \
--config_name=MINITAUR_GYM_CONFIG
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function


import os
from absl import app
from absl import flags
import ars
import config_ars

FLAGS = flags.FLAGS
flags.DEFINE_string('logdir', None, 'The directory to write the log file.')
flags.DEFINE_string('config_name', None, 'The name of the config dictionary')


def run_ars(config, logdir):

  env = config["env"]()
  ob_dim = env.observation_space.shape[0]
  ac_dim = env.action_space.shape[0]

  # set policy parameters. Possible filters: 'MeanStdFilter' for v2, 'NoFilter' for v1.
  policy_params = {
      'type': 'linear',
      'ob_filter': config['filter'],
      'ob_dim': ob_dim,
      'ac_dim': ac_dim
  }

  ARS = ars.ARSLearner(
      env_callback=config['env'],
      policy_params=policy_params,
      num_deltas=config['num_directions'],
      deltas_used=config['deltas_used'],
      step_size=config['step_size'],
      delta_std=config['delta_std'],
      logdir=logdir,
      rollout_length=config['rollout_length'],
      shift=config['shift'],
      params=config,
      seed=config['seed'])

  return ARS.train(config['num_iterations'])


def main(argv):
  del argv  # Unused.
  config = getattr(config_ars, FLAGS.config_name)
  run_ars(config=config, logdir=FLAGS.logdir)


if __name__ == '__main__':
  flags.mark_flag_as_required('logdir')
  flags.mark_flag_as_required('config_name')
  app.run(main)
