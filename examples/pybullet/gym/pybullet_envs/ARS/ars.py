"""Internal implementation of the Augmented Random Search method."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
os.sys.path.insert(0,currentdir)

from concurrent import futures
import copy
import os
import time
import gym
import numpy as np
import logz
import utils
import optimizers
#from google3.pyglib import gfile
import policies
import shared_noise
import utility

class Worker(object):
  """Object class for parallel rollout generation."""

  def __init__(self,
               env_seed,
               env_callback,
               policy_params=None,
               deltas=None,
               rollout_length=1000,
               delta_std=0.02):

    # initialize OpenAI environment for each worker
    self.env = env_callback()
    self.env.seed(env_seed)

    # each worker gets access to the shared noise table
    # with independent random streams for sampling
    # from the shared noise table.
    self.deltas = shared_noise.SharedNoiseTable(deltas, env_seed + 7)
    self.policy_params = policy_params
    if policy_params['type'] == 'linear':
      self.policy = policies.LinearPolicy(policy_params)
    else:
      raise NotImplementedError

    self.delta_std = delta_std
    self.rollout_length = rollout_length

  def get_weights_plus_stats(self):
    """
        Get current policy weights and current statistics of past states.
        """
    assert self.policy_params['type'] == 'linear'
    return self.policy.get_weights_plus_stats()

  def rollout(self, shift=0., rollout_length=None):
    """Performs one rollout of maximum length rollout_length.

    At each time-step it substracts shift from the reward.
    """

    if rollout_length is None:
      rollout_length = self.rollout_length

    total_reward = 0.
    steps = 0

    ob = self.env.reset()
    for i in range(rollout_length):
      action = self.policy.act(ob)
      ob, reward, done, _ = self.env.step(action)
      steps += 1
      total_reward += (reward - shift)
      if done:
        break

    return total_reward, steps

  def do_rollouts(self, w_policy, num_rollouts=1, shift=1, evaluate=False):
    """
        Generate multiple rollouts with a policy parametrized by w_policy.
        """
    print('Doing {} rollouts'.format(num_rollouts))
    rollout_rewards, deltas_idx = [], []
    steps = 0

    for i in range(num_rollouts):

      if evaluate:
        self.policy.update_weights(w_policy)
        deltas_idx.append(-1)

        # set to false so that evaluation rollouts are not used for updating state statistics
        self.policy.update_filter = False

        # for evaluation we do not shift the rewards (shift = 0) and we use the
        # default rollout length (1000 for the MuJoCo locomotion tasks)
        reward, r_steps = self.rollout(
            shift=0., rollout_length=self.rollout_length)
        rollout_rewards.append(reward)

      else:
        idx, delta = self.deltas.get_delta(w_policy.size)

        delta = (self.delta_std * delta).reshape(w_policy.shape)
        deltas_idx.append(idx)

        # set to true so that state statistics are updated
        self.policy.update_filter = True

        # compute reward and number of timesteps used for positive perturbation rollout
        self.policy.update_weights(w_policy + delta)
        pos_reward, pos_steps = self.rollout(shift=shift)

        # compute reward and number of timesteps used for negative pertubation rollout
        self.policy.update_weights(w_policy - delta)
        neg_reward, neg_steps = self.rollout(shift=shift)
        steps += pos_steps + neg_steps

        rollout_rewards.append([pos_reward, neg_reward])

    return {
        'deltas_idx': deltas_idx,
        'rollout_rewards': rollout_rewards,
        'steps': steps
    }

  def stats_increment(self):
    self.policy.observation_filter.stats_increment()
    return

  def get_weights(self):
    return self.policy.get_weights()

  def get_filter(self):
    return self.policy.observation_filter

  def sync_filter(self, other):
    self.policy.observation_filter.sync(other)
    return


class ARSLearner(object):
  """
    Object class implementing the ARS algorithm.
    """

  def __init__(self,
               env_callback,
               policy_params=None,
               num_workers=32,
               num_deltas=320,
               deltas_used=320,
               delta_std=0.02,
               logdir=None,
               rollout_length=1000,
               step_size=0.01,
               shift='constant zero',
               params=None,
               seed=123):

    logz.configure_output_dir(logdir)
    # params_to_save = copy.deepcopy(params)
    # params_to_save['env'] = None
    # logz.save_params(params_to_save)
    utility.save_config(params, logdir)
    env = env_callback()

    self.timesteps = 0
    self.action_size = env.action_space.shape[0]
    self.ob_size = env.observation_space.shape[0]
    self.num_deltas = num_deltas
    self.deltas_used = deltas_used
    self.rollout_length = rollout_length
    self.step_size = step_size
    self.delta_std = delta_std
    self.logdir = logdir
    self.shift = shift
    self.params = params
    self.max_past_avg_reward = float('-inf')
    self.num_episodes_used = float('inf')

    # create shared table for storing noise
    print('Creating deltas table.')
    deltas = shared_noise.create_shared_noise()
    self.deltas = shared_noise.SharedNoiseTable(deltas, seed=seed + 3)
    print('Created deltas table.')

    # initialize workers with different random seeds
    print('Initializing workers.')
    self.num_workers = num_workers
    self.workers = [
        Worker(
            seed + 7 * i,
            env_callback=env_callback,
            policy_params=policy_params,
            deltas=deltas,
            rollout_length=rollout_length,
            delta_std=delta_std) for i in range(num_workers)
    ]

    # initialize policy
    if policy_params['type'] == 'linear':
      self.policy = policies.LinearPolicy(policy_params)
      self.w_policy = self.policy.get_weights()
    else:
      raise NotImplementedError

    # initialize optimization algorithm
    self.optimizer = optimizers.SGD(self.w_policy, self.step_size)
    print('Initialization of ARS complete.')

  def aggregate_rollouts(self, num_rollouts=None, evaluate=False):
    """
        Aggregate update step from rollouts generated in parallel.
        """

    if num_rollouts is None:
      num_deltas = self.num_deltas
    else:
      num_deltas = num_rollouts

    results_one = []  #rollout_ids_one
    results_two = []  #rollout_ids_two

    t1 = time.time()
    num_rollouts = int(num_deltas / self.num_workers)
#     if num_rollouts > 0:
#       with futures.ThreadPoolExecutor(
#           max_workers=self.num_workers) as executor:
#         workers = [
#             executor.submit(
#                 worker.do_rollouts,
#                 self.w_policy,
#                 num_rollouts=num_rollouts,
#                 shift=self.shift,
#                 evaluate=evaluate) for worker in self.workers
#         ]
#         for worker in futures.as_completed(workers):
#           results_one.append(worker.result())
#
#       workers = [
#           executor.submit(
#               worker.do_rollouts,
#               self.w_policy,
#               num_rollouts=1,
#               shift=self.shift,
#               evaluate=evaluate)
#           for worker in self.workers[:(num_deltas % self.num_workers)]
#       ]
#       for worker in futures.as_completed(workers):
#         results_two.append(worker.result())

    # parallel generation of rollouts
    rollout_ids_one = [
        worker.do_rollouts(
            self.w_policy,
            num_rollouts=num_rollouts,
            shift=self.shift,
            evaluate=evaluate) for worker in self.workers
    ]

    rollout_ids_two = [
        worker.do_rollouts(
            self.w_policy, num_rollouts=1, shift=self.shift, evaluate=evaluate)
        for worker in self.workers[:(num_deltas % self.num_workers)]
    ]
    results_one = rollout_ids_one
    results_two = rollout_ids_two
# gather results

    rollout_rewards, deltas_idx = [], []

    for result in results_one:
      if not evaluate:
        self.timesteps += result['steps']
      deltas_idx += result['deltas_idx']
      rollout_rewards += result['rollout_rewards']

    for result in results_two:
      if not evaluate:
        self.timesteps += result['steps']
      deltas_idx += result['deltas_idx']
      rollout_rewards += result['rollout_rewards']

    deltas_idx = np.array(deltas_idx)
    rollout_rewards = np.array(rollout_rewards, dtype=np.float64)

    print('Maximum reward of collected rollouts:', rollout_rewards.max())
    info_dict = {
        "max_reward": rollout_rewards.max()
    }
    t2 = time.time()

    print('Time to generate rollouts:', t2 - t1)

    if evaluate:
      return rollout_rewards

    # select top performing directions if deltas_used < num_deltas
    max_rewards = np.max(rollout_rewards, axis=1)
    if self.deltas_used > self.num_deltas:
      self.deltas_used = self.num_deltas

    idx = np.arange(max_rewards.size)[max_rewards >= np.percentile(
        max_rewards, 100 * (1 - (self.deltas_used / self.num_deltas)))]
    deltas_idx = deltas_idx[idx]
    rollout_rewards = rollout_rewards[idx, :]

    # normalize rewards by their standard deviation
    rollout_rewards /= np.std(rollout_rewards)

    t1 = time.time()
    # aggregate rollouts to form g_hat, the gradient used to compute SGD step
    g_hat, count = utils.batched_weighted_sum(
        rollout_rewards[:, 0] - rollout_rewards[:, 1],
        (self.deltas.get(idx, self.w_policy.size) for idx in deltas_idx),
        batch_size=500)
    g_hat /= deltas_idx.size
    t2 = time.time()
    print('time to aggregate rollouts', t2 - t1)
    return g_hat, info_dict

  def train_step(self):
    """
        Perform one update step of the policy weights.
        """

    g_hat, info_dict = self.aggregate_rollouts()
    print('Euclidean norm of update step:', np.linalg.norm(g_hat))
    self.w_policy -= self.optimizer._compute_step(g_hat).reshape(
        self.w_policy.shape)
    return info_dict

  def train(self, num_iter):

    start = time.time()
    for i in range(num_iter):

      t1 = time.time()
      info_dict = self.train_step()
      t2 = time.time()
      print('total time of one step', t2 - t1)
      print('iter ', i, ' done')

      # record statistics every 10 iterations
      if ((i) % 10 == 0):

        rewards = self.aggregate_rollouts(num_rollouts=8, evaluate=True)
        w = self.workers[0].get_weights_plus_stats()

        checkpoint_filename = os.path.join(
            self.logdir, 'lin_policy_plus_{:03d}.npz'.format(i))
        print('Save checkpoints to {}...', checkpoint_filename)
        checkpoint_file = open(checkpoint_filename, 'w')
        np.savez(checkpoint_file, w)
        print('End save checkpoints.')
        print(sorted(self.params.items()))
        logz.log_tabular('Time', time.time() - start)
        logz.log_tabular('Iteration', i + 1)
        logz.log_tabular('AverageReward', np.mean(rewards))
        logz.log_tabular('StdRewards', np.std(rewards))
        logz.log_tabular('MaxRewardRollout', np.max(rewards))
        logz.log_tabular('MinRewardRollout', np.min(rewards))
        logz.log_tabular('timesteps', self.timesteps)
        logz.dump_tabular()

      t1 = time.time()
      # get statistics from all workers
      for j in range(self.num_workers):
        self.policy.observation_filter.update(self.workers[j].get_filter())
      self.policy.observation_filter.stats_increment()

      # make sure master filter buffer is clear
      self.policy.observation_filter.clear_buffer()
      # sync all workers
      #filter_id = ray.put(self.policy.observation_filter)
      setting_filters_ids = [
          worker.sync_filter(self.policy.observation_filter)
          for worker in self.workers
      ]
      # waiting for sync of all workers
      #ray.get(setting_filters_ids)

      increment_filters_ids = [
          worker.stats_increment() for worker in self.workers
      ]
      # waiting for increment of all workers
      #ray.get(increment_filters_ids)
      t2 = time.time()
      print('Time to sync statistics:', t2 - t1)

    return info_dict
