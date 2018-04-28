"""
Policy class for computing action from weights and observation vector.
Horia Mania --- hmania@berkeley.edu
Aurelia Guy
Benjamin Recht
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
import filter


class Policy(object):

  def __init__(self, policy_params):

    self.ob_dim = policy_params['ob_dim']
    self.ac_dim = policy_params['ac_dim']
    self.weights = np.empty(0)

    # a filter for updating statistics of the observations and normalizing
    # inputs to the policies
    self.observation_filter = filter.get_filter(
        policy_params['ob_filter'], shape=(self.ob_dim,))
    self.update_filter = True

  def update_weights(self, new_weights):
    self.weights[:] = new_weights[:]
    return

  def get_weights(self):
    return self.weights

  def get_observation_filter(self):
    return self.observation_filter

  def act(self, ob):
    raise NotImplementedError

  def copy(self):
    raise NotImplementedError


class LinearPolicy(Policy):
  """
    Linear policy class that computes action as <w, ob>.
    """

  def __init__(self, policy_params, update_filter=True):
    Policy.__init__(self, policy_params)
    self.weights = np.zeros(self.ac_dim * self.ob_dim, dtype=np.float64)
    if "weights" in policy_params:
      self.weights = policy_params["weights"]
    if "mean" in policy_params:
      self.observation_filter.mean = policy_params["mean"]
    if "std" in policy_params:
      self.observation_filter.std = policy_params["std"]
    self.update_filter = update_filter

  def act(self, ob):
    ob = self.observation_filter(ob, update=self.update_filter)
    matrix_weights = np.reshape(self.weights, (self.ac_dim, self.ob_dim))
    return np.clip(np.dot(matrix_weights, ob), -1.0, 1.0)

  def get_weights_plus_stats(self):

    mu, std = self.observation_filter.get_stats()
    aux = np.asarray([self.weights, mu, std])
    return aux
