# Lint as: python3
"""The system to log and manage all metrics."""

import logging
from typing import Any, Callable, Dict, Sequence, Text, Union
import numpy as np

from pybullet_envs.minitaur.envs_v2.evaluation import metric as metric_lib


def _merge_dict_throw_on_duplicates(
    dict_1: Dict[Text, Any],
    dict_2: Dict[Text, Any],
):
  """Merge the contents of dict_2 to dict_1.

  Args:
    dict_1: The dictionary to merge into.
    dict_2: The dictionary to merge from.

  Raises:
    KeyError: if duplicated keys are found.
  """
  for key, val in dict_2.items():
    if key in dict_1:
      raise KeyError(f"Duplicate key: {key} found in both "
                     f"dictionaries: {dict_1} {dict_2}.")
    dict_1[key] = val


def common_stats(x: Sequence[Union[float, Sequence[float]]],
                 flatten_array=False):
  out = x
  if flatten_array:
    # Deals with array of arrays.
    out = np.concatenate(x).flatten()
  return {
      "mean": np.mean(out),
      "max": np.max(out),
      "min": np.min(out),
      "std": np.std(out),
  }


class MetricLogger(object):
  """The central system to manage all metrics."""

  def __init__(self, ignore_duplicate_metrics: bool = True):
    """Initializes the system.

    Args:
      ignore_duplicate_metrics: Don't throw error when users want to recreate
        the same metric.
    """
    self._metric_reporters = {}
    self._ignore_duplicate_metrics = ignore_duplicate_metrics

  def reset_episode(self):
    """Resets all metric reporters's internal buffer.

    Will be called by the gym env during reset.
    """
    for metric in self._metric_reporters.values():
      metric.reset_episode()

  def create_metric(
      self,
      name: Text,
      scope: metric_lib.MetricScope,
      single_ep_aggregator: Callable[[Sequence[Any]], Any],
      multi_ep_aggregator: Callable[[Sequence[Any]], Dict[Text, Any]],
  ) -> metric_lib.MetricCore:
    """Creates a new metric.

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
      multi_ep_aggregator: The function to process multi-episode metric values.
        We assume the inputs to the function is a list of per episode metric
        values, i.e. each element of the list is the output from the
        single_ep_aggregator.

    Returns:
      A MetricCore which can be used to report values of interest.
    """
    if name in self._metric_reporters:
      if self._ignore_duplicate_metrics:
        logging.warning("Trying to create an existing metric: %s", name)
        name = self._get_valid_duplicate_name(name)
        logging.warning("Changed duplicate metric to new name: %s", name)
      else:
        raise ValueError(f"Duplicated metrics found: {name}")

    self._metric_reporters[name] = metric_lib.MetricReporter(
        name, scope, single_ep_aggregator, multi_ep_aggregator)
    return self._metric_reporters[name]

  def create_scalar_metric(
      self,
      name: Text,
      scope: metric_lib.MetricScope,
      single_ep_aggregator: Callable[[Sequence[Any]], Any],
  ) -> metric_lib.MetricCore:
    """Shortcut to create a metric for scalar variables."""
    return self.create_metric(
        name,
        scope,
        single_ep_aggregator,
        multi_ep_aggregator=common_stats,
    )

  def _get_valid_duplicate_name(self, original_name: Text) -> Text:
    counter = 1
    test_name = "{}_duplicate_{}".format(original_name, str(counter))
    while test_name in self._metric_reporters:
      counter += 1
      test_name = "{}_duplicate_{}".format(original_name, str(counter))
    return test_name

  def get_episode_metrics(self) -> Dict[Text, Any]:
    """Return all metrics registered in the logger for the current episode."""
    ep_stats = {}
    for metric in self._metric_reporters.values():
      _merge_dict_throw_on_duplicates(ep_stats, metric.get_episode_metric())
    return ep_stats

  def get_multi_episode_metrics(
      self, episodic_metrics: Dict[Text, Sequence[Any]]) -> Dict[Text, Any]:
    """Processes the aggregated metrics over many episodes.

    Will not be affected by reset_episode, since we take multi-episode data as
    inputs.

    Args:
      episodic_metrics: The per episode metrics. For each key in the inputs, we
        expect at least one metric reporter can process the corresponding value,
        which contains a list of episodic metrics.

    Returns:
      The processed multi-episode metrics.
    """
    stats = {}
    for metric in self._metric_reporters.values():
      _merge_dict_throw_on_duplicates(
          stats, metric.get_multi_ep_metric(episodic_metrics))
    return stats
