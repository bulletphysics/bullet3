try:
  import tensorflow.compat.v1 as tf
except Exception:
  import tensorflow as tf
import numpy as np
import os

try:
  xavier_initializer = tf.contrib.layers.xavier_initializer()
except Exception:
  xavier_initializer = None


def disable_gpu():
  os.environ["CUDA_VISIBLE_DEVICES"] = '-1'
  return


def var_shape(x):
  out = [k.value for k in x.get_shape()]
  assert all(isinstance(a, int) for a in out), "shape function assumes that shape is fully known"
  return out


def intprod(x):
  return int(np.prod(x))


def numel(x):
  n = intprod(var_shape(x))
  return n


def flat_grad(loss, var_list, grad_ys=None):
  grads = tf.gradients(loss, var_list, grad_ys)
  return tf.concat([tf.reshape(grad, [numel(v)]) for (v, grad) in zip(var_list, grads)], axis=0)


def fc_net(input, layers_sizes, activation, reuse=None,
           flatten=False):  # build fully connected network
  curr_tf = input
  for i, size in enumerate(layers_sizes):
    with tf.variable_scope(str(i), reuse=reuse):
      curr_tf = tf.layers.dense(inputs=curr_tf,
                                units=size,
                                kernel_initializer=xavier_initializer,
                                activation=activation if i < len(layers_sizes) - 1 else None)
  if flatten:
    assert layers_sizes[-1] == 1
    curr_tf = tf.reshape(curr_tf, [-1])

  return curr_tf


def copy(sess, src, dst):
  assert len(src) == len(dst)
  sess.run(list(map(lambda v: v[1].assign(v[0]), zip(src, dst))))
  return


def flat_grad(loss, var_list):
  grads = tf.gradients(loss, var_list)
  return tf.concat(axis=0,
                   values=[tf.reshape(grad, [numel(v)]) for (v, grad) in zip(var_list, grads)])


def calc_logp_gaussian(x_tf, mean_tf, std_tf):
  dim = tf.to_float(tf.shape(x_tf)[-1])

  if mean_tf is None:
    diff_tf = x_tf
  else:
    diff_tf = x_tf - mean_tf

  logp_tf = -0.5 * tf.reduce_sum(tf.square(diff_tf / std_tf), axis=-1)
  logp_tf += -0.5 * dim * np.log(2 * np.pi) - tf.reduce_sum(tf.log(std_tf), axis=-1)

  return logp_tf


def calc_bound_loss(x_tf, bound_min, bound_max):
  # penalty for violating bounds
  violation_min = tf.minimum(x_tf - bound_min, 0)
  violation_max = tf.maximum(x_tf - bound_max, 0)
  violation = tf.reduce_sum(tf.square(violation_min), axis=-1) + tf.reduce_sum(
      tf.square(violation_max), axis=-1)
  loss = 0.5 * tf.reduce_mean(violation)
  return loss


class SetFromFlat(object):

  def __init__(self, sess, var_list, dtype=tf.float32):
    assigns = []
    shapes = list(map(var_shape, var_list))
    total_size = np.sum([intprod(shape) for shape in shapes])

    self.sess = sess
    self.theta = tf.placeholder(dtype, [total_size])
    start = 0
    assigns = []

    for (shape, v) in zip(shapes, var_list):
      size = intprod(shape)
      assigns.append(tf.assign(v, tf.reshape(self.theta[start:start + size], shape)))
      start += size

    self.op = tf.group(*assigns)

    return

  def __call__(self, theta):
    self.sess.run(self.op, feed_dict={self.theta: theta})
    return


class GetFlat(object):

  def __init__(self, sess, var_list):
    self.sess = sess
    self.op = tf.concat(axis=0, values=[tf.reshape(v, [numel(v)]) for v in var_list])
    return

  def __call__(self):
    return self.sess.run(self.op)
