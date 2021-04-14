"""State estimator."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import abc


class StateEstimatorBase(object):  # pytype: disable=ignored-metaclass
  """Estimates the unmeasurable state of the robot."""

  __metaclass__ = abc.ABCMeta

  @abc.abstractmethod
  def reset(self, current_time):
    pass

  @abc.abstractmethod
  def update(self, current_time):
    pass
