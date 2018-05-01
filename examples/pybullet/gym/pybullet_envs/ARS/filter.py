# Code in this file is copied and adapted from
# https://github.com/ray-project/ray/blob/master/python/ray/rllib/utils/filter.py


from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np


class Filter(object):
    """Processes input, possibly statefully."""

    def update(self, other, *args, **kwargs):
        """Updates self with "new state" from other filter."""
        raise NotImplementedError

    def copy(self):
        """Creates a new object with same state as self.

        Returns:
            copy (Filter): Copy of self"""
        raise NotImplementedError

    def sync(self, other):
        """Copies all state from other filter to self."""
        raise NotImplementedError


class NoFilter(Filter):
    def __init__(self, *args):
        pass

    def __call__(self, x, update=True):
        return np.asarray(x, dtype = np.float64)

    def update(self, other, *args, **kwargs):
        pass

    def copy(self):
        return self

    def sync(self, other):
        pass

    def stats_increment(self):
        pass

    def clear_buffer(self):
        pass

    def get_stats(self):
        return 0, 1

    @property
    def mean(self):
        return 0

    @property
    def var(self):
        return 1

    @property
    def std(self):
        return 1



# http://www.johndcook.com/blog/standard_deviation/
class RunningStat(object):

    def __init__(self, shape=None):
        self._n = 0
        self._M = np.zeros(shape, dtype = np.float64)
        self._S = np.zeros(shape,  dtype = np.float64)
        self._M2 = np.zeros(shape,  dtype = np.float64)

    def copy(self):
        other = RunningStat()
        other._n = self._n
        other._M = np.copy(self._M)
        other._S = np.copy(self._S)
        return other

    def push(self, x):
        x = np.asarray(x)
        # Unvectorized update of the running statistics.
        assert x.shape == self._M.shape, ("x.shape = {}, self.shape = {}"
                                          .format(x.shape, self._M.shape))
        n1 = self._n
        self._n += 1
        if self._n == 1:
            self._M[...] = x
        else:
            delta = x - self._M
            deltaM2 = np.square(x) - self._M2
            self._M[...] += delta / self._n
            self._S[...] += delta * delta * n1 / self._n
            

    def update(self, other):
        n1 = self._n
        n2 = other._n
        n = n1 + n2
        delta = self._M - other._M
        delta2 = delta * delta
        M = (n1 * self._M + n2 * other._M) / n
        S = self._S + other._S + delta2 * n1 * n2 / n
        self._n = n
        self._M = M
        self._S = S

    def __repr__(self):
        return '(n={}, mean_mean={}, mean_std={})'.format(
            self.n, np.mean(self.mean), np.mean(self.std))

    @property
    def n(self):
        return self._n

    @property
    def mean(self):
        return self._M

    @property
    def var(self):
        return self._S / (self._n - 1) if self._n > 1 else np.square(self._M)

    @property
    def std(self):
        return np.sqrt(self.var)

    @property
    def shape(self):
        return self._M.shape


class MeanStdFilter(Filter):
    """Keeps track of a running mean for seen states"""

    def __init__(self, shape, demean=True, destd=True):
        self.shape = shape
        self.demean = demean
        self.destd = destd
        self.rs = RunningStat(shape)
        # In distributed rollouts, each worker sees different states.
        # The buffer is used to keep track of deltas amongst all the
        # observation filters.

        self.buffer = RunningStat(shape)

        self.mean = np.zeros(shape, dtype = np.float64)
        self.std = np.ones(shape, dtype = np.float64)

    def clear_buffer(self):
        self.buffer = RunningStat(self.shape)
        return

    def update(self, other, copy_buffer=False):
        """Takes another filter and only applies the information from the
        buffer.

        Using notation `F(state, buffer)`
        Given `Filter1(x1, y1)` and `Filter2(x2, yt)`,
        `update` modifies `Filter1` to `Filter1(x1 + yt, y1)`
        If `copy_buffer`, then `Filter1` is modified to
        `Filter1(x1 + yt, yt)`.
        """
        self.rs.update(other.buffer)
        if copy_buffer:
            self.buffer = other.buffer.copy()
        return 

    def copy(self):
        """Returns a copy of Filter."""
        other = MeanStdFilter(self.shape)
        other.demean = self.demean
        other.destd = self.destd
        other.rs = self.rs.copy()
        other.buffer = self.buffer.copy()
        return other

    def sync(self, other):
        """Syncs all fields together from other filter.

        Using notation `F(state, buffer)`
        Given `Filter1(x1, y1)` and `Filter2(x2, yt)`,
        `sync` modifies `Filter1` to `Filter1(x2, yt)`
        """
        assert other.shape == self.shape, "Shapes don't match!"
        self.demean = other.demean
        self.destd = other.destd
        self.rs = other.rs.copy()
        self.buffer = other.buffer.copy()
        return

    def __call__(self, x, update=True):
        x = np.asarray(x, dtype = np.float64)
        if update:
            if len(x.shape) == len(self.rs.shape) + 1:
                # The vectorized case.
                for i in range(x.shape[0]):
                    self.rs.push(x[i])
                    self.buffer.push(x[i])
            else:
                # The unvectorized case.
                self.rs.push(x)
                self.buffer.push(x)
        if self.demean:
            x = x - self.mean
        if self.destd:
            x = x / (self.std + 1e-8)
        return x

    def stats_increment(self):
        self.mean = self.rs.mean
        self.std = self.rs.std

        # Set values for std less than 1e-7 to +inf to avoid 
        # dividing by zero. State elements with zero variance
        # are set to zero as a result. 
        self.std[self.std < 1e-7] = float("inf") 
        return

    def get_stats(self):
        return self.rs.mean, (self.rs.std + 1e-8)

    def __repr__(self):
        return 'MeanStdFilter({}, {}, {}, {}, {}, {})'.format(
            self.shape, self.demean,
            self.rs, self.buffer)

    
def get_filter(filter_config, shape = None):
    if filter_config == "MeanStdFilter":
        return MeanStdFilter(shape)
    elif filter_config == "NoFilter":
        return NoFilter()
    else:
        raise Exception("Unknown observation_filter: " +
                        str(filter_config))


def test_running_stat():
    for shp in ((), (3,), (3, 4)):
        li = []
        rs = RunningStat(shp)
        for _ in range(5):
            val = np.random.randn(*shp)
            rs.push(val)
            li.append(val)
            m = np.mean(li, axis=0)
            assert np.allclose(rs.mean, m)
            v = np.square(m) if (len(li) == 1) else np.var(li, ddof=1, axis=0)
            assert np.allclose(rs.var, v)


def test_combining_stat():
    for shape in [(), (3,), (3, 4)]:
        li = []
        rs1 = RunningStat(shape)
        rs2 = RunningStat(shape)
        rs = RunningStat(shape)
        for _ in range(5):
            val = np.random.randn(*shape)
            rs1.push(val)
            rs.push(val)
            li.append(val)
        for _ in range(9):
            rs2.push(val)
            rs.push(val)
            li.append(val)
        rs1.update(rs2)
        assert np.allclose(rs.mean, rs1.mean)
        assert np.allclose(rs.std, rs1.std)


test_running_stat()
test_combining_stat()
