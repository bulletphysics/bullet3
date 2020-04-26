# Copyright 2017 The TensorFlow Agents Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Network definitions for the PPO algorithm."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import collections
import functools
import operator

try:
  import tensorflow.compat.v1 as tf
except Exception:
  import tensorflow as tf

NetworkOutput = collections.namedtuple('NetworkOutput', 'policy, mean, logstd, value, state')


def feed_forward_gaussian(config, action_size, observations, unused_length, state=None):
  """Independent feed forward networks for policy and value.

  The policy network outputs the mean action and the log standard deviation
  is learned as independent parameter vector.

  Args:
    config: Configuration object.
    action_size: Length of the action vector.
    observations: Sequences of observations.
    unused_length: Batch of sequence lengths.
    state: Batch of initial recurrent states.

  Returns:
    NetworkOutput tuple.
  """
  mean_weights_initializer = tf.contrib.layers.variance_scaling_initializer(
      factor=config.init_mean_factor)
  logstd_initializer = tf.random_normal_initializer(config.init_logstd, 1e-10)
  flat_observations = tf.reshape(observations, [
      tf.shape(observations)[0],
      tf.shape(observations)[1],
      functools.reduce(operator.mul,
                       observations.shape.as_list()[2:], 1)
  ])
  with tf.variable_scope('policy'):
    x = flat_observations
    for size in config.policy_layers:
      x = tf.contrib.layers.fully_connected(x, size, tf.nn.relu)
    mean = tf.contrib.layers.fully_connected(x,
                                             action_size,
                                             tf.tanh,
                                             weights_initializer=mean_weights_initializer)
    logstd = tf.get_variable('logstd', mean.shape[2:], tf.float32, logstd_initializer)
    logstd = tf.tile(logstd[None, None],
                     [tf.shape(mean)[0], tf.shape(mean)[1]] + [1] * (mean.shape.ndims - 2))
  with tf.variable_scope('value'):
    x = flat_observations
    for size in config.value_layers:
      x = tf.contrib.layers.fully_connected(x, size, tf.nn.relu)
    value = tf.contrib.layers.fully_connected(x, 1, None)[..., 0]
  mean = tf.check_numerics(mean, 'mean')
  logstd = tf.check_numerics(logstd, 'logstd')
  value = tf.check_numerics(value, 'value')
  policy = tf.contrib.distributions.MultivariateNormalDiag(mean, tf.exp(logstd))
  return NetworkOutput(policy, mean, logstd, value, state)


def recurrent_gaussian(config, action_size, observations, length, state=None):
  """Independent recurrent policy and feed forward value networks.

  The policy network outputs the mean action and the log standard deviation
  is learned as independent parameter vector. The last policy layer is
  recurrent and uses a GRU cell.

  Args:
    config: Configuration object.
    action_size: Length of the action vector.
    observations: Sequences of observations.
    length: Batch of sequence lengths.
    state: Batch of initial recurrent states.

  Returns:
    NetworkOutput tuple.
  """
  mean_weights_initializer = tf.contrib.layers.variance_scaling_initializer(
      factor=config.init_mean_factor)
  logstd_initializer = tf.random_normal_initializer(config.init_logstd, 1e-10)
  cell = tf.contrib.rnn.GRUBlockCell(config.policy_layers[-1])
  flat_observations = tf.reshape(observations, [
      tf.shape(observations)[0],
      tf.shape(observations)[1],
      functools.reduce(operator.mul,
                       observations.shape.as_list()[2:], 1)
  ])
  with tf.variable_scope('policy'):
    x = flat_observations
    for size in config.policy_layers[:-1]:
      x = tf.contrib.layers.fully_connected(x, size, tf.nn.relu)
    x, state = tf.nn.dynamic_rnn(cell, x, length, state, tf.float32)
    mean = tf.contrib.layers.fully_connected(x,
                                             action_size,
                                             tf.tanh,
                                             weights_initializer=mean_weights_initializer)
    logstd = tf.get_variable('logstd', mean.shape[2:], tf.float32, logstd_initializer)
    logstd = tf.tile(logstd[None, None],
                     [tf.shape(mean)[0], tf.shape(mean)[1]] + [1] * (mean.shape.ndims - 2))
  with tf.variable_scope('value'):
    x = flat_observations
    for size in config.value_layers:
      x = tf.contrib.layers.fully_connected(x, size, tf.nn.relu)
    value = tf.contrib.layers.fully_connected(x, 1, None)[..., 0]
  mean = tf.check_numerics(mean, 'mean')
  logstd = tf.check_numerics(logstd, 'logstd')
  value = tf.check_numerics(value, 'value')
  policy = tf.contrib.distributions.MultivariateNormalDiag(mean, tf.exp(logstd))
  # assert state.shape.as_list()[0] is not None
  return NetworkOutput(policy, mean, logstd, value, state)
