"""Loads a DDPG agent without too much external dependencies
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import collections
import numpy as np
import tensorflow as tf

import sonnet as snt
from agents import actor_net

class SimpleAgent():
  def __init__(
      self,
      session,
      ckpt_path,
      actor_layer_size,
      observation_size=(31,),
      action_size=8,
  ):
    self._ckpt_path = ckpt_path
    self._actor_layer_size = actor_layer_size
    self._observation_size = observation_size
    self._action_size = action_size
    self._session = session
    self._build()

  def _build(self):
    self._agent_net = actor_net.ActorNetwork(self._actor_layer_size, self._action_size)
    self._o_t = tf.placeholder(tf.float32, (31,))
    with tf.name_scope('Act'):
      batch_o_t = snt.nest.pack_iterable_as(
          self._o_t,
          snt.nest.map(
              lambda x: tf.expand_dims(x, 0),
              snt.nest.flatten_iterable(self._o_t)))
      self._action = self._agent_net(batch_o_t)
      saver = tf.train.Saver()
      saver.restore(
          sess=self._session,
          save_path=self._ckpt_path)

  def __call__(self, observation):
    out_action = self._session.run(self._action, feed_dict={self._o_t: observation})
    return out_action[0]
