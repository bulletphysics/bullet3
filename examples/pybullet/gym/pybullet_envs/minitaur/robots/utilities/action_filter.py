"""Two types of filters which can be applied to policy output sequences.

1. Simple exponential filter
2. Butterworth filter - lowpass or bandpass

The implementation of the butterworth filter follows scipy's lfilter
https://github.com/scipy/scipy/blob/v1.2.1/scipy/signal/signaltools.py

We re-implement the logic in order to explicitly manage the y states

The filter implements::
       a[0]*y[n] = b[0]*x[n] + b[1]*x[n-1] + ... + b[M]*x[n-M]
                             - a[1]*y[n-1] - ... - a[N]*y[n-N]

We assume M == N.
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import collections
from absl import logging
import gin
import numpy as np
from scipy.signal import butter

ACTION_FILTER_ORDER = 2
ACTION_FILTER_LOW_CUT = 0.0
ACTION_FILTER_HIGH_CUT = 4.0


@gin.configurable
class ActionFilter(object):
  """Implements a generic lowpass or bandpass action filter."""

  def __init__(self, a, b, order, num_joints, ftype='lowpass'):
    """Initializes filter.

    Either one per joint or same for all joints.

    Args:
      a: filter output history coefficients
      b: filter input coefficients
      order: filter order
      num_joints: robot DOF
      ftype: filter type. 'lowpass' or 'bandpass'
    """
    self.num_joints = num_joints
    if isinstance(a, list):
      self.a = a
      self.b = b
    else:
      self.a = [a]
      self.b = [b]

    # Either a set of parameters per joint must be specified as a list
    # Or one filter is applied to every joint
    if not ((len(self.a) == len(self.b) == num_joints) or (
        len(self.a) == len(self.b) == 1)):
      raise ValueError('Incorrect number of filter values specified')

    # Normalize by a[0]
    for i in range(len(self.a)):
      self.b[i] /= self.a[i][0]
      self.a[i] /= self.a[i][0]

    # Convert single filter to same format as filter per joint
    if len(self.a) == 1:
      self.a *= num_joints
      self.b *= num_joints
    self.a = np.stack(self.a)
    self.b = np.stack(self.b)

    if ftype == 'bandpass':
      assert len(self.b[0]) == len(self.a[0]) == 2 * order + 1
      self.hist_len = 2 * order
    elif ftype == 'lowpass':
      assert len(self.b[0]) == len(self.a[0]) == order + 1
      self.hist_len = order
    else:
      raise ValueError('%s filter type not supported' % (ftype))

    logging.info('Filter shapes: a: %s, b: %s', self.a.shape, self.b.shape)
    logging.info('Filter type:%s', ftype)

    self.yhist = collections.deque(maxlen=self.hist_len)
    self.xhist = collections.deque(maxlen=self.hist_len)
    self.reset()

  def reset(self):
    """Resets the history buffers to 0."""
    self.yhist.clear()
    self.xhist.clear()
    for _ in range(self.hist_len):
      self.yhist.appendleft(np.zeros((self.num_joints, 1)))
      self.xhist.appendleft(np.zeros((self.num_joints, 1)))

  def filter(self, x):
    """Returns filtered x."""
    xs = np.concatenate(list(self.xhist), axis=-1)
    ys = np.concatenate(list(self.yhist), axis=-1)
    y = np.multiply(x, self.b[:, 0]) + np.sum(
        np.multiply(xs, self.b[:, 1:]), axis=-1) - np.sum(
            np.multiply(ys, self.a[:, 1:]), axis=-1)
    self.xhist.appendleft(x.reshape((self.num_joints, 1)).copy())
    self.yhist.appendleft(y.reshape((self.num_joints, 1)).copy())
    return y

  def init_history(self, x):
    x = np.expand_dims(x, axis=-1)
    for i in range(self.hist_len):
      self.xhist[i] = x
      self.yhist[i] = x
    return


@gin.configurable
class ActionFilterButter(ActionFilter):
  """Butterworth filter."""

  def __init__(self,
               lowcut=None,
               highcut=None,
               sampling_rate=None,
               order=ACTION_FILTER_ORDER,
               num_joints=None):
    """Initializes a butterworth filter.

    Either one per joint or same for all joints.

    Args:
      lowcut: list of strings defining the low cutoff frequencies.
        The list must contain either 1 element (same filter for all joints)
        or num_joints elements
        0 for lowpass, > 0 for bandpass. Either all values must be 0
        or all > 0
      highcut: list of strings defining the high cutoff frequencies.
        The list must contain either 1 element (same filter for all joints)
        or num_joints elements
        All must be > 0
      sampling_rate: frequency of samples in Hz
      order: filter order
      num_joints: robot DOF
    """
    self.lowcut = ([float(x) for x in lowcut]
                   if lowcut is not None else [ACTION_FILTER_LOW_CUT])
    self.highcut = ([float(x) for x in highcut]
                    if highcut is not None else [ACTION_FILTER_HIGH_CUT])
    if len(self.lowcut) != len(self.highcut):
      raise ValueError('Number of lowcut and highcut filter values should '
                       'be the same')

    if sampling_rate is None:
      raise ValueError('sampling_rate should be provided.')

    if num_joints is None:
      raise ValueError('num_joints should be provided.')

    if np.any(self.lowcut):
      if not np.all(self.lowcut):
        raise ValueError('All the filters must be of the same type: '
                         'lowpass or bandpass')
      self.ftype = 'bandpass'
    else:
      self.ftype = 'lowpass'

    a_coeffs = []
    b_coeffs = []
    for i, (l, h) in enumerate(zip(self.lowcut, self.highcut)):
      if h <= 0.0:
        raise ValueError('Highcut must be > 0')

      b, a = self.butter_filter(l, h, sampling_rate, order)
      logging.info(
          'Butterworth filter: joint: %d, lowcut: %f, highcut: %f, '
          'sampling rate: %d, order: %d, num joints: %d', i, l, h,
          sampling_rate, order, num_joints)
      b_coeffs.append(b)
      a_coeffs.append(a)

    super(ActionFilterButter, self).__init__(
        a_coeffs, b_coeffs, order, num_joints, self.ftype)

  def butter_filter(self, lowcut, highcut, fs, order=5):
    """Returns the coefficients of a butterworth filter.

    If lowcut = 0, the function returns the coefficients of a low pass filter.
    Otherwise, the coefficients of a band pass filter are returned.
    Highcut should be > 0

    Args:
      lowcut: low cutoff frequency
      highcut: high cutoff frequency
      fs: sampling rate
      order: filter order
    Return:
      b, a: parameters of a butterworth filter
    """
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    if low:
      b, a = butter(order, [low, high], btype='band')
    else:
      b, a = butter(order, [high], btype='low')
    return b, a


class ActionFilterExp(ActionFilter):
  """Filter by way of simple exponential smoothing.

  y = alpha * x + (1 - alpha) * previous_y
  """

  def __init__(self, alpha, num_joints):
    """Initialize the filter.

    Args:
      alpha: list of strings defining the alphas.
        The list must contain either 1 element (same filter for all joints)
        or num_joints elements
        0 < alpha <= 1
      num_joints: robot DOF
    """
    self.alphas = [float(x) for x in alpha]
    logging.info('Exponential filter: alpha: %d', self.alphas)

    a_coeffs = []
    b_coeffs = []
    for a in self.alphas:
      a_coeffs.append(np.asarray([1., a - 1.]))
      b_coeffs.append(np.asarray([a, 0]))

    order = 1
    self.ftype = 'lowpass'

    super(ActionFilterExp, self).__init__(
        a_coeffs, b_coeffs, order, num_joints, self.ftype)
