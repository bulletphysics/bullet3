"""A class for combining multiple state estimators."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from typing import Any, Sequence

import gin

from pybullet_envs.minitaur.agents.baseline_controller import state_estimator


@gin.configurable
class MultiStateEstimator(state_estimator.StateEstimatorBase):
  """Combine multiple state estimators.


  This class can be used to combine multiple state estimators into one. For
  example, one can use the COMVelocityEstimator to estimate the com velocity
  and COMHeightEstimator to estimate the com height.

  """

  def __init__(
      self,
      robot: Any,
      state_estimators: Sequence[state_estimator.StateEstimatorBase],
  ):
    self._robot = robot
    self._state_estimators = state_estimators
    self.reset(0)

  def reset(self, current_time):
    for single_state_estimator in self._state_estimators:
      single_state_estimator.reset(current_time)

  def update(self, current_time):
    for single_state_estimator in self._state_estimators:
      single_state_estimator.update(current_time)

  def __getattr__(self, attr):
    for single_state_estimator in self._state_estimators:
      if hasattr(single_state_estimator, attr):
        return getattr(single_state_estimator, attr)
    raise ValueError(
        "{} is not found in any of the state estimators".format(attr))

