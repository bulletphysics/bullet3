import numpy as np
import copy
from pybullet_utils.logger import Logger
import inspect as inspect
from pybullet_envs.deep_mimic.env.env import Env
import pybullet_utils.math_util as MathUtil


class ReplayBuffer(object):
  TERMINATE_KEY = 'terminate'
  PATH_START_KEY = 'path_start'
  PATH_END_KEY = 'path_end'

  def __init__(self, buffer_size):
    assert buffer_size > 0

    self.buffer_size = buffer_size
    self.total_count = 0
    self.buffer_head = 0
    self.buffer_tail = MathUtil.INVALID_IDX
    self.num_paths = 0
    self._sample_buffers = dict()
    self.buffers = None

    self.clear()
    return

  def sample(self, n):
    curr_size = self.get_current_size()
    assert curr_size > 0

    idx = np.empty(n, dtype=int)
    # makes sure that the end states are not sampled
    for i in range(n):
      while True:
        curr_idx = np.random.randint(0, curr_size, size=1)[0]
        curr_idx += self.buffer_tail
        curr_idx = np.mod(curr_idx, self.buffer_size)

        if not self.is_path_end(curr_idx):
          break
      idx[i] = curr_idx

    return idx

  def sample_filtered(self, n, key):
    assert key in self._sample_buffers
    curr_buffer = self._sample_buffers[key]
    idx = curr_buffer.sample(n)
    return idx

  def count_filtered(self, key):
    curr_buffer = self._sample_buffers[key]
    return curr_buffer.count

  def get(self, key, idx):
    return self.buffers[key][idx]

  def get_all(self, key):
    return self.buffers[key]

  def get_idx_filtered(self, key):
    assert key in self._sample_buffers
    curr_buffer = self._sample_buffers[key]
    idx = curr_buffer.slot_to_idx[:curr_buffer.count]
    return idx

  def get_path_start(self, idx):
    return self.buffers[self.PATH_START_KEY][idx]

  def get_path_end(self, idx):
    return self.buffers[self.PATH_END_KEY][idx]

  def get_pathlen(self, idx):
    is_array = isinstance(idx, np.ndarray) or isinstance(idx, list)
    if not is_array:
      idx = [idx]

    n = len(idx)
    start_idx = self.get_path_start(idx)
    end_idx = self.get_path_end(idx)
    pathlen = np.empty(n, dtype=int)

    for i in range(n):
      curr_start = start_idx[i]
      curr_end = end_idx[i]
      if curr_start < curr_end:
        curr_len = curr_end - curr_start
      else:
        curr_len = self.buffer_size - curr_start + curr_end
      pathlen[i] = curr_len

    if not is_array:
      pathlen = pathlen[0]

    return pathlen

  def is_valid_path(self, idx):
    start_idx = self.get_path_start(idx)
    valid = start_idx != MathUtil.INVALID_IDX
    return valid

  def store(self, path):
    start_idx = MathUtil.INVALID_IDX
    n = path.pathlength()

    if (n > 0):
      assert path.is_valid()

      if path.check_vals():
        if self.buffers is None:
          self._init_buffers(path)

        idx = self._request_idx(n + 1)
        self._store_path(path, idx)
        self._add_sample_buffers(idx)

        self.num_paths += 1
        self.total_count += n + 1
        start_idx = idx[0]
      else:
        Logger.print2('Invalid path data value detected')

    return start_idx

  def clear(self):
    self.buffer_head = 0
    self.buffer_tail = MathUtil.INVALID_IDX
    self.num_paths = 0

    for key in self._sample_buffers:
      self._sample_buffers[key].clear()
    return

  def get_next_idx(self, idx):
    next_idx = np.mod(idx + 1, self.buffer_size)
    return next_idx

  def is_terminal_state(self, idx):
    terminate_flags = self.buffers[self.TERMINATE_KEY][idx]
    terminate = terminate_flags != Env.Terminate.Null.value
    is_end = self.is_path_end(idx)
    terminal_state = np.logical_and(terminate, is_end)
    return terminal_state

  def check_terminal_flag(self, idx, flag):
    terminate_flags = self.buffers[self.TERMINATE_KEY][idx]
    terminate = terminate_flags == flag.value
    return terminate

  def is_path_end(self, idx):
    is_end = self.buffers[self.PATH_END_KEY][idx] == idx
    return is_end

  def add_filter_key(self, key):
    assert self.get_current_size() == 0
    if key not in self._sample_buffers:
      self._sample_buffers[key] = SampleBuffer(self.buffer_size)
    return

  def get_current_size(self):
    if self.buffer_tail == MathUtil.INVALID_IDX:
      return 0
    elif self.buffer_tail < self.buffer_head:
      return self.buffer_head - self.buffer_tail
    else:
      return self.buffer_size - self.buffer_tail + self.buffer_head

  def _check_flags(self, key, flags):
    return (flags & key) == key

  def _add_sample_buffers(self, idx):
    flags = self.buffers['flags']
    for key in self._sample_buffers:
      curr_buffer = self._sample_buffers[key]
      filter_idx = [
          i for i in idx if (self._check_flags(key, flags[i]) and not self.is_path_end(i))
      ]
      curr_buffer.add(filter_idx)
    return

  def _free_sample_buffers(self, idx):
    for key in self._sample_buffers:
      curr_buffer = self._sample_buffers[key]
      curr_buffer.free(idx)
    return

  def _init_buffers(self, path):
    self.buffers = dict()
    self.buffers[self.PATH_START_KEY] = MathUtil.INVALID_IDX * np.ones(self.buffer_size, dtype=int)
    self.buffers[self.PATH_END_KEY] = MathUtil.INVALID_IDX * np.ones(self.buffer_size, dtype=int)

    for key in dir(path):
      val = getattr(path, key)
      if not key.startswith('__') and not inspect.ismethod(val):
        if key == self.TERMINATE_KEY:
          self.buffers[self.TERMINATE_KEY] = np.zeros(shape=[self.buffer_size], dtype=int)
        else:
          val_type = type(val[0])
          is_array = val_type == np.ndarray
          if is_array:
            shape = [self.buffer_size, val[0].shape[0]]
            dtype = val[0].dtype
          else:
            shape = [self.buffer_size]
            dtype = val_type

          self.buffers[key] = np.zeros(shape, dtype=dtype)
    return

  def _request_idx(self, n):
    assert n + 1 < self.buffer_size  # bad things can happen if path is too long

    remainder = n
    idx = []

    start_idx = self.buffer_head
    while remainder > 0:
      end_idx = np.minimum(start_idx + remainder, self.buffer_size)
      remainder -= (end_idx - start_idx)

      free_idx = list(range(start_idx, end_idx))
      self._free_idx(free_idx)
      idx += free_idx
      start_idx = 0

    self.buffer_head = (self.buffer_head + n) % self.buffer_size
    return idx

  def _free_idx(self, idx):
    assert (idx[0] <= idx[-1])
    n = len(idx)
    if self.buffer_tail != MathUtil.INVALID_IDX:
      update_tail = idx[0] <= idx[-1] and idx[0] <= self.buffer_tail and idx[-1] >= self.buffer_tail
      update_tail |= idx[0] > idx[-1] and (idx[0] <= self.buffer_tail or
                                           idx[-1] >= self.buffer_tail)

      if update_tail:
        i = 0
        while i < n:
          curr_idx = idx[i]
          if self.is_valid_path(curr_idx):
            start_idx = self.get_path_start(curr_idx)
            end_idx = self.get_path_end(curr_idx)
            pathlen = self.get_pathlen(curr_idx)

            if start_idx < end_idx:
              self.buffers[self.PATH_START_KEY][start_idx:end_idx + 1] = MathUtil.INVALID_IDX
              self._free_sample_buffers(list(range(start_idx, end_idx + 1)))
            else:
              self.buffers[self.PATH_START_KEY][start_idx:self.buffer_size] = MathUtil.INVALID_IDX
              self.buffers[self.PATH_START_KEY][0:end_idx + 1] = MathUtil.INVALID_IDX
              self._free_sample_buffers(list(range(start_idx, self.buffer_size)))
              self._free_sample_buffers(list(range(0, end_idx + 1)))

            self.num_paths -= 1
            i += pathlen + 1
            self.buffer_tail = (end_idx + 1) % self.buffer_size
          else:
            i += 1
    else:
      self.buffer_tail = idx[0]
    return

  def _store_path(self, path, idx):
    n = path.pathlength()
    for key, data in self.buffers.items():
      if key != self.PATH_START_KEY and key != self.PATH_END_KEY and key != self.TERMINATE_KEY:
        val = getattr(path, key)
        val_len = len(val)
        assert val_len == n or val_len == n + 1
        data[idx[:val_len]] = val

    self.buffers[self.TERMINATE_KEY][idx] = path.terminate.value
    self.buffers[self.PATH_START_KEY][idx] = idx[0]
    self.buffers[self.PATH_END_KEY][idx] = idx[-1]
    return


class SampleBuffer(object):

  def __init__(self, size):
    self.idx_to_slot = np.empty(shape=[size], dtype=int)
    self.slot_to_idx = np.empty(shape=[size], dtype=int)
    self.count = 0
    self.clear()
    return

  def clear(self):
    self.idx_to_slot.fill(MathUtil.INVALID_IDX)
    self.slot_to_idx.fill(MathUtil.INVALID_IDX)
    self.count = 0
    return

  def is_valid(self, idx):
    return self.idx_to_slot[idx] != MathUtil.INVALID_IDX

  def get_size(self):
    return self.idx_to_slot.shape[0]

  def add(self, idx):
    for i in idx:
      if not self.is_valid(i):
        new_slot = self.count
        assert new_slot >= 0

        self.idx_to_slot[i] = new_slot
        self.slot_to_idx[new_slot] = i
        self.count += 1
    return

  def free(self, idx):
    for i in idx:
      if self.is_valid(i):
        slot = self.idx_to_slot[i]
        last_slot = self.count - 1
        last_idx = self.slot_to_idx[last_slot]

        self.idx_to_slot[last_idx] = slot
        self.slot_to_idx[slot] = last_idx
        self.idx_to_slot[i] = MathUtil.INVALID_IDX
        self.slot_to_idx[last_slot] = MathUtil.INVALID_IDX
        self.count -= 1
    return

  def sample(self, n):
    if self.count > 0:
      slots = np.random.randint(0, self.count, size=n)
      idx = self.slot_to_idx[slots]
    else:
      idx = np.empty(shape=[0], dtype=int)
    return idx

  def check_consistency(self):
    valid = True
    if self.count < 0:
      valid = False

    if valid:
      for i in range(self.get_size()):
        if self.is_valid(i):
          s = self.idx_to_slot[i]
          if self.slot_to_idx[s] != i:
            valid = False
            break

        s2i = self.slot_to_idx[i]
        if s2i != MathUtil.INVALID_IDX:
          i2s = self.idx_to_slot[s2i]
          if i2s != i:
            valid = False
            break

    count0 = np.sum(self.idx_to_slot == MathUtil.INVALID_IDX)
    count1 = np.sum(self.slot_to_idx == MathUtil.INVALID_IDX)
    valid &= count0 == count1
    return valid
