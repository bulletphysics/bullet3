import numpy as np
import copy
try:
  import tensorflow.compat.v1 as tf
except Exception:
  import tensorflow as tf
from pybullet_envs.deep_mimic.learning.normalizer import Normalizer


class TFNormalizer(Normalizer):

  def __init__(self, sess, scope, size, groups_ids=None, eps=0.02, clip=np.inf):
    self.sess = sess
    self.scope = scope
    super().__init__(size, groups_ids, eps, clip)

    with tf.variable_scope(self.scope):
      self._build_resource_tf()
    return

  # initialze count when loading saved values so that things don't change to quickly during updates
  def load(self):
    self.count = self.count_tf.eval()[0]
    self.mean = self.mean_tf.eval()
    self.std = self.std_tf.eval()
    self.mean_sq = self.calc_mean_sq(self.mean, self.std)
    return

  def update(self):
    super().update()
    self._update_resource_tf()
    return

  def set_mean_std(self, mean, std):
    super().set_mean_std(mean, std)
    self._update_resource_tf()
    return

  def normalize_tf(self, x):
    norm_x = (x - self.mean_tf) / self.std_tf
    norm_x = tf.clip_by_value(norm_x, -self.clip, self.clip)
    return norm_x

  def unnormalize_tf(self, norm_x):
    x = norm_x * self.std_tf + self.mean_tf
    return x

  def _build_resource_tf(self):
    self.count_tf = tf.get_variable(dtype=tf.int32,
                                    name='count',
                                    initializer=np.array([self.count], dtype=np.int32),
                                    trainable=False)
    self.mean_tf = tf.get_variable(dtype=tf.float32,
                                   name='mean',
                                   initializer=self.mean.astype(np.float32),
                                   trainable=False)
    self.std_tf = tf.get_variable(dtype=tf.float32,
                                  name='std',
                                  initializer=self.std.astype(np.float32),
                                  trainable=False)

    self.count_ph = tf.get_variable(dtype=tf.int32, name='count_ph', shape=[1])
    self.mean_ph = tf.get_variable(dtype=tf.float32, name='mean_ph', shape=self.mean.shape)
    self.std_ph = tf.get_variable(dtype=tf.float32, name='std_ph', shape=self.std.shape)

    self._update_op = tf.group(self.count_tf.assign(self.count_ph),
                               self.mean_tf.assign(self.mean_ph), self.std_tf.assign(self.std_ph))
    return

  def _update_resource_tf(self):
    feed = {
        self.count_ph: np.array([self.count], dtype=np.int32),
        self.mean_ph: self.mean,
        self.std_ph: self.std
    }
    self.sess.run(self._update_op, feed_dict=feed)
    return
