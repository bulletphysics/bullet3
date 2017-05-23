"""Loads a DDPG agent without too much external dependencies
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import collections
import numpy as np
import tensorflow as tf
import pdb

class SimpleAgent():
  def __init__(
      self,
      session,
      ckpt_path,
      actor_layer_size,
      observation_dim=31
  ):
    self._ckpt_path = ckpt_path
    self._session = session
    self._observation_dim = observation_dim
    self._build()

  def _build(self):
    saver = tf.train.import_meta_graph(self._ckpt_path + '.meta')
    saver.restore(
        sess=self._session,
        save_path=self._ckpt_path)
    self._action = tf.get_collection('action_op')[0]
    self._obs = tf.get_collection('observation_placeholder')[0]

  def __call__(self, observation):
    feed_dict={self._obs: observation}
    out_action = self._session.run(self._action, feed_dict=feed_dict)
    return out_action[0]
