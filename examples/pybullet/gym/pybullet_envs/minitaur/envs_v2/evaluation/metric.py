"""The metric reporting system used in our env.

In the gym environment, there are many variables or quantities that can help
researchers to debug, evaluate policy performance. Such quantities may
include: Motor torques for quadrupeds, even when they are controlled in
POSITION mode; Distance to walls while a wheeled robot is navigating the
indoor environment. Often, these metrics are private variables (or can only be
computed from private variables). To expose the user interested metrics
outside the environment's observations, we designed this Metric system that
can be invoked in any modules (robot, tasks, sensors) inside the gym env.
"""

import enum
import logging
from typing import Any, Callable, Dict, Sequence, Text
import gin


@gin.constants_from_enum
class MetricScope(enum.Enum):
  """The supported scope of metrics."""
  # The performance metrics.
  PERFORMANCE = 1,

  # The debug metrics for diagnostic purposes.
  DEBUG = 2,

  # The safety metrics.
  SAFETY = 3,

  # The statistics of episodes in metric format.
  STATISTIC = 4,


class MetricCore(object):
  """Aggregates values of interest to compute statistics."""

  def __init__(
      self,
      name: Text,
      scope: MetricScope,
      single_ep_aggregator: Callable[[Sequence[Any]], Any],
      multi_ep_aggregator: Callable[[Sequence[Any]], Dict[Text, Any]],
  ):
    """Initializes the class.

    Args:
      name: The name of the metric, for example "motor_torques",
        "distance_to_wall", etc. The full name of the metric will have scope
        name in the prefix, i.e. "scope/name".
      scope: The scope of this metric. Most metric should be for DEBUG purpose.
        The scope name will be added to the final name of metric in this way:
          "scope/name", which is standarded format for Tensorboard to group
          named variables.
      single_ep_aggregator: The function to process all aggregated metric
        values. The derived MetricReporter (see below) will implements
        reset_episode() which clears the episode data, and will be called during
        env.reset().
      multi_ep_aggregator: The functions to process multi-episode metric values.
        We assume the inputs to the functions is a list of per episode metric
        values, i.e. each element of the list is the output from the
        single_ep_aggregator.
    """
    self._name = scope.name + "/" + name
    self._single_ep_aggregator = single_ep_aggregator
    self._multi_ep_aggregator = multi_ep_aggregator
    self._episode_data = []

  def report(self, metric_value: Any):
    """Stores the reported metric in the internal buffer.

    Args:
      metric_value: The metric we are interested to report.
    """
    self._episode_data.append(metric_value)


class MetricReporter(MetricCore):
  """Reports the metric using the provided aggregator functions."""

  def get_episode_metric(self) -> Dict[Text, Sequence[Any]]:
    """Processes and returns episode metric values.

    Returns:
      Aggregated metrics for the current episode.
    """
    if self._episode_data:
      return {self._name: self._single_ep_aggregator(self._episode_data)}
    else:
      return {}

  def get_multi_ep_metric(
      self, episodic_metric: Dict[Text, Sequence[Any]]) -> Dict[Text, Any]:
    """Processes the aggregated metrics over many episodes.

    Will not be affected by reset_episode, since we take multi-episode data as
    inputs.

    Args:
      episodic_metric: The per episode metrics. We expect the inputs to contain
        the same key as self._name, and that the value is a list of metric
        values computed using self.get_episode_metrc().

    Returns:
      The processed multi-episode metrics.
    """
    if self._name not in episodic_metric:
      logging.warning(
          "The inputs does not contain the key for the current metric: %s",
          self._name)
      return {}
    outputs = {}
    for key, val in self._multi_ep_aggregator(
        episodic_metric[self._name]).items():
      outputs[self._name + "_" + key] = val
    return outputs

  def reset_episode(self):
    """Clears the episode data stored.

    Will be invoked during env.reset(). This will effect how get_episode_metric
    gets computed.

    """
    self._episode_data = []
