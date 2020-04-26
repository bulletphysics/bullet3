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
"""Mock algorithm for testing reinforcement learning code."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
try:
  import tensorflow.compat.v1 as tf
except Exception:
  import tensorflow as tf


class MockAlgorithm(object):
  """Produce random actions and empty summaries."""

  def __init__(self, envs):
    """Produce random actions and empty summaries.

    Args:
      envs: List of in-graph environments.
    """
    self._envs = envs

  def begin_episode(self, unused_agent_indices):
    return tf.constant('')

  def perform(self, agent_indices, unused_observ):
    shape = (tf.shape(agent_indices)[0],) + self._envs[0].action_space.shape
    low = self._envs[0].action_space.low
    high = self._envs[0].action_space.high
    action = tf.random_uniform(shape) * (high - low) + low
    return action, tf.constant('')

  def experience(self, unused_agent_indices, *unused_transition):
    return tf.constant('')

  def end_episode(self, unused_agent_indices):
    return tf.constant('')
