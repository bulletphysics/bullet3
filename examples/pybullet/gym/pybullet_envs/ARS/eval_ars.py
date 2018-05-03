"""
blaze run -c opt //experimental/users/jietan/ARS:eval_ars -- \
--logdir=/cns/ij-d/home/jietan/experiment/ARS/ars_react_nr01.191950338.191950550/ \
--checkpoint=lin_policy_plus_990.npz \
--num_rollouts=10
"""


from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os, inspect
import time

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
os.sys.path.insert(0,currentdir)

from absl import app
from absl import flags

import pdb
import os
import numpy as np
import gym
import config_ars
import utility
import policies

FLAGS = flags.FLAGS

flags.DEFINE_string('logdir', None, 'The path of the checkpoint.')
flags.DEFINE_string('checkpoint', None, 'The file name of the checkpoint.')
flags.DEFINE_integer('num_rollouts', 1, 'The number of rollouts.')


def main(argv):
  del argv  # Unused.

  print('loading and building expert policy')
  checkpoint_file = os.path.join(FLAGS.logdir, FLAGS.checkpoint)
  lin_policy = np.load(checkpoint_file, encoding='bytes')
  lin_policy = lin_policy.items()[0][1]

  M = lin_policy[0]
  # mean and std of state vectors estimated online by ARS.
  mean = lin_policy[1]
  std = lin_policy[2]

  config = utility.load_config(FLAGS.logdir)
  print("config=",config)
  env = config['env'](hard_reset=True, render=True)
  ob_dim = env.observation_space.shape[0]
  ac_dim = env.action_space.shape[0]

  # set policy parameters. Possible filters: 'MeanStdFilter' for v2, 'NoFilter' for v1.
  policy_params = {
      'type': 'linear',
      'ob_filter': config['filter'],
      'ob_dim': ob_dim,
      'ac_dim': ac_dim,
      "weights": M,
      "mean": mean,
      "std": std,
  }
  policy = policies.LinearPolicy(policy_params, update_filter=False)
  returns = []
  observations = []
  actions = []
  for i in range(FLAGS.num_rollouts):
    print('iter', i)
    obs = env.reset()
    done = False
    totalr = 0.
    steps = 0
    while not done:
      action = policy.act(obs)
      observations.append(obs)
      actions.append(action)

      obs, r, done, _ = env.step(action)
      time.sleep(1./100.)
      totalr += r
      steps += 1
      if steps % 100 == 0:
        print('%i/%i' % (steps, config['rollout_length']))
      if steps >= config['rollout_length']:
        break
    returns.append(totalr)

  print('returns', returns)
  print('mean return', np.mean(returns))
  print('std of return', np.std(returns))


if __name__ == '__main__':
  flags.mark_flag_as_required('logdir')
  flags.mark_flag_as_required('checkpoint')
  app.run(main)
