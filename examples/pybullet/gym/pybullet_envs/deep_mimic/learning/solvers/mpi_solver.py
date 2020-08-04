from mpi4py import MPI
try:
  import tensorflow.compat.v1 as tf
except Exception:
  import tensorflow as tf
  
import numpy as np
import pybullet_envs.deep_mimic.learning.tf_util as TFUtil
import pybullet_utils.math_util as MathUtil
import pybullet_utils.mpi_util as MPIUtil
from pybullet_utils.logger import Logger

from pybullet_envs.deep_mimic.learning.solvers.solver import Solver


class MPISolver(Solver):
  CHECK_SYNC_ITERS = 1000

  def __init__(self, sess, optimizer, vars):
    super().__init__(vars)
    self.sess = sess
    self.optimizer = optimizer
    self._build_grad_feed(vars)
    self._update = optimizer.apply_gradients(zip(self._grad_tf_list, self.vars))
    self._set_flat_vars = TFUtil.SetFromFlat(sess, self.vars)
    self._get_flat_vars = TFUtil.GetFlat(sess, self.vars)

    self.iter = 0
    grad_dim = self._calc_grad_dim()
    self._flat_grad = np.zeros(grad_dim, dtype=np.float32)
    self._global_flat_grad = np.zeros(grad_dim, dtype=np.float32)

    return

  def get_stepsize(self):
    return self.optimizer._learning_rate_tensor.eval()

  def update(self, grads=None, grad_scale=1.0):
    if grads is not None:
      self._flat_grad = MathUtil.flatten(grads)
    else:
      self._flat_grad.fill(0)
    return self.update_flatgrad(self._flat_grad, grad_scale)

  def update_flatgrad(self, flat_grad, grad_scale=1.0):
    if self.iter % self.CHECK_SYNC_ITERS == 0:
      assert self.check_synced(), Logger.print2('Network parameters desynchronized')

    if grad_scale != 1.0:
      flat_grad *= grad_scale

    MPI.COMM_WORLD.Allreduce(flat_grad, self._global_flat_grad, op=MPI.SUM)
    self._global_flat_grad /= MPIUtil.get_num_procs()

    self._load_flat_grad(self._global_flat_grad)
    self.sess.run([self._update], self._grad_feed)
    self.iter += 1

    return

  def sync(self):
    vars = self._get_flat_vars()
    MPIUtil.bcast(vars)
    self._set_flat_vars(vars)
    return

  def check_synced(self):
    synced = True
    if self._is_root():
      vars = self._get_flat_vars()
      MPIUtil.bcast(vars)
    else:
      vars_local = self._get_flat_vars()
      vars_root = np.empty_like(vars_local)
      MPIUtil.bcast(vars_root)
      synced = (vars_local == vars_root).all()
    return synced

  def _is_root(self):
    return MPIUtil.is_root_proc()

  def _build_grad_feed(self, vars):
    self._grad_tf_list = []
    self._grad_buffers = []
    for v in self.vars:
      shape = v.get_shape()
      grad = np.zeros(shape)
      grad_tf = tf.placeholder(tf.float32, shape=shape)
      self._grad_buffers.append(grad)
      self._grad_tf_list.append(grad_tf)

    self._grad_feed = dict({g_tf: g for g_tf, g in zip(self._grad_tf_list, self._grad_buffers)})

    return

  def _calc_grad_dim(self):
    grad_dim = 0
    for grad in self._grad_buffers:
      grad_dim += grad.size
    return grad_dim

  def _load_flat_grad(self, flat_grad):
    start = 0
    for g in self._grad_buffers:
      size = g.size
      np.copyto(g, np.reshape(flat_grad[start:start + size], g.shape))
      start += size
    return
