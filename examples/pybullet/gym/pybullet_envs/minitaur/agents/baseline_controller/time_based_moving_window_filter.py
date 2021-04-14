"""A moving-window filter for smoothing the signals within certain time interval."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function


import gin
import numpy as np


@gin.configurable
class TimeBasedMovingWindowFilter:
  """A moving-window filter for smoothing the signals within certain time interval."""

  def __init__(
      self,
      filter_window: float = 0.1,
  ):
    """Initializes the class.

    Args:
      filter_window: The filtering window (in time) used to smooth the input
        signal.
    """
    self._filter_window = filter_window
    self.reset()

  def reset(self):
    self._timestamp_buffer = []
    self._value_buffer = []

  def calculate_average(self, new_value, timestamp):
    """Compute the filtered signals based on the time-based moving window."""
    self._timestamp_buffer.append(timestamp)
    self._value_buffer.append(new_value)

    while len(self._value_buffer) > 1:
      if self._timestamp_buffer[
          0] < timestamp - self._filter_window:
        self._timestamp_buffer.pop(0)
        self._value_buffer.pop(0)
      else:
        break
    return np.mean(self._value_buffer, axis=0)
