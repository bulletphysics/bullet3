"""Helper class and functions to make computing meteric statistics easier."""
from typing import Any, Sequence

import numpy as np


class MetricStats(object):
  """Helper class to make computing meteric statistics easier to manage."""

  def __init__(self, data: Sequence[Any]):
    if None or not list(data):
      raise ValueError("Input data for ComputeMetricStats cannot be empty.")
    self._data = np.asarray(data).flatten()

  @property
  def avg(self):
    return np.mean(self._data)

  @property
  def min(self):
    return np.min(self._data)

  @property
  def max(self):
    return np.max(self._data)

  @property
  def sum(self):
    return np.sum(self._data)
