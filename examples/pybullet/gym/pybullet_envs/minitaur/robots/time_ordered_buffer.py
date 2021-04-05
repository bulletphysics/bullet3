# Lint as: python3
"""The common class to manage the a stream of past data."""

import collections
from typing import Any, List, Sequence, Union
import dataclasses
import gin
import numpy as np


@dataclasses.dataclass
class BufferTuple:
  value_0: Any
  value_1: Any
  coeff: float


FloatOrArray = Union[float, Sequence[float]]
BufferTupleOrArray = Union[BufferTuple, Sequence[BufferTuple]]
TIME_IDX = 0
VALUE_IDX = 1


@gin.configurable
class TimeOrderedBuffer(object):
  """A buffer to hold and extract history data."""

  def __init__(self,
               max_buffer_timespan: float,
               error_on_timestamp_reversal: bool = True,
               error_on_duplicate_timestamp: bool = True,
               replace_value_on_duplicate_timestamp: bool = False,
               ):
    """Initializes the class.

    Args:
      max_buffer_timespan: Maximum amount of buffer by time to keep.
      error_on_timestamp_reversal: Whether to throw error if inverted timestamps
        are found.
      error_on_duplicate_timestamp: Whether to throw error if the incoming
        data has the same timestamp as the latest timestamp in the buffer.
      replace_value_on_duplicate_timestamp: Whether to keep the new value when
        a duplicate timestamp has occurred. This only applies if we are not
        throwing an error on duplicate timestamps.
    """
    if max_buffer_timespan < 0:
      raise ValueError(
          "Invalid max_buffer_timespan: {}".format(max_buffer_timespan))
    self._max_buffer_timespan = max_buffer_timespan
    self._error_on_timestamp_reversal = error_on_timestamp_reversal
    self._error_on_duplicate_timestamp = error_on_duplicate_timestamp
    self._replace_value_on_duplicate_timestamp = (
        replace_value_on_duplicate_timestamp)
    # TODO(tsangwei): Look for a ring buffer implementation.
    self._buffer = collections.deque()

  def reset(self):
    self._buffer.clear()

  def _compute_coeff(self,
                     newer_time: float,
                     older_time: float,
                     target_time: float,
                     ) -> float:
    """Compute the coefficient value between the two timestamps.

    Args:
      newer_time: The newer timestamp.
      older_time: The older timestamp.
      target_time: Target timestamp that is between the newer and older values.

    Returns:
      The coefficient as a float.
    """
    coeff = 0.0
    # Prevents divide by 0 error.
    if newer_time != older_time:
      assert older_time <= target_time <= newer_time
      coeff = (newer_time - target_time) / (newer_time - older_time)
    return coeff

  def _pack_data(self,
                 obs_newer: Any,
                 obs_older: Any,
                 target_time: float,
                 ) -> BufferTuple:
    """Packs up buffer data as BufferTuple dataclass.

    Args:
      obs_newer: Timestamp and value of newer observation.
      obs_older: Timestamp and value of older observation.
      target_time: Target timestamp of the observation we are looking for.

    Returns:
      BufferTuple dataclass.
    """
    coeff = self._compute_coeff(newer_time=obs_newer[TIME_IDX],
                                older_time=obs_older[TIME_IDX],
                                target_time=target_time)
    return BufferTuple(value_0=obs_newer[VALUE_IDX],
                       value_1=obs_older[VALUE_IDX],
                       coeff=coeff)

  def _find_values_at(self, timestamp_targets: Sequence[float]) -> List[Any]:
    """Get the lower/upper bound values for given target timestamp.

    Args:
      timestamp_targets: Actual timestamp value to match against.

    Returns:
      List of BufferTuple dataclass.
    """
    results = [None] * len(timestamp_targets)
    oldest_obs = self._buffer[0]
    latest_obs = self._buffer[-1]

    search_start_idx = None
    search_end_idx = None

    # Check to make sure we do not try to search for values outside of the
    # current buffer.
    for i in range(len(timestamp_targets)):
      # Oldest observation have the smallest timestamp.
      if timestamp_targets[i] <= oldest_obs[TIME_IDX]:
        results[i] = self._pack_data(obs_newer=oldest_obs,
                                     obs_older=oldest_obs,
                                     target_time=timestamp_targets[i])
      elif timestamp_targets[i] >= latest_obs[TIME_IDX]:
        results[i] = self._pack_data(obs_newer=latest_obs,
                                     obs_older=latest_obs,
                                     target_time=timestamp_targets[i])
      else:
        if search_end_idx is None:
          search_end_idx = i
        search_start_idx = i

    if search_end_idx is not None:
      results = self._walkthrough_buffer(timestamp_targets=timestamp_targets,
                                         search_start_idx=search_start_idx,
                                         search_end_idx=search_end_idx,
                                         results=results)

    return results

  def _walkthrough_buffer(self,
                          timestamp_targets: List[float],
                          search_start_idx: int,
                          search_end_idx: int,
                          results: List[BufferTuple],
                          ) -> List[BufferTuple]:
    """Actual method to walk through the buffer looking for requested values.

    Args:
      timestamp_targets: List of timestamps to search for in buffer.
      search_start_idx: Index number for timestamp_targets to start searching
        from.
      search_end_idx: Index number for timestamp_targets to stop searching at.
      results: List of BufferTuple values that covers out of bound results.

    Returns:
      List of BufferTuple values.
    """
    value_older = None
    target_idx = search_start_idx
    target_timestamp = timestamp_targets[target_idx]
    value_older = self._buffer[0]

    # Searching from oldest timestamp to latest timestamp.
    for value_newer in self._buffer:
      # Catch edge case scenario where multiple timestamp targets are between
      # the same two buffer timestamps. (b/157104935)
      while value_newer[TIME_IDX] >= target_timestamp:
        # Catch special edge case scenario if using older_obs_blender method.
        obs_older = value_newer if (
            value_newer[TIME_IDX] == target_timestamp) else value_older
        results[target_idx] = self._pack_data(obs_newer=value_newer,
                                              obs_older=obs_older,
                                              target_time=target_timestamp)
        if target_idx - 1 >= search_end_idx:
          target_idx -= 1
          target_timestamp = timestamp_targets[target_idx]
        else:
          return results
      value_older = value_newer

    return results

  def add(self, timestamp: float, value: Any):
    """Inserts timestamp and value into buffer.

    Args:
      timestamp: Timestamp of the data value.
      value: Data value to be saved into the buffer.
    """
    if self._buffer:
      last_timestamp = self._buffer[-1][TIME_IDX]
      if last_timestamp == timestamp:
        if (not np.array_equal(self._buffer[-1][VALUE_IDX], value) and
            self._error_on_duplicate_timestamp):
          raise ValueError("Duplicate timestamp detected: {}".format(timestamp))
        else:
          # Duplicate message detected.
          if self._replace_value_on_duplicate_timestamp:
            self._buffer[-1] = (timestamp, value)
          return
      if last_timestamp > timestamp and self._error_on_timestamp_reversal:
        raise ValueError(
            "Time reversal detected: new timestamp is {} vs last timestamp {}"
            .format(timestamp, last_timestamp))
      # Dropping old buffer data that exceed buffer timespan limit and making
      # sure the buffer does not go empty.
      while (len(self._buffer) > 1 and self._max_buffer_timespan <
             (timestamp - self._buffer[1][TIME_IDX])):
        self._buffer.popleft()
    self._buffer.append((timestamp, value))

  def get_delayed_value(self, latency: FloatOrArray) -> BufferTupleOrArray:
    """Retrieves value in the history buffer according to latency.

    Finds the closest pair of values that are some time (i.e. latency) ago from
    the most recent timestamp. Suppose the history buffer looks like this:

    [(0, val_x),...,(0.6, val_k-1), (0.7, val_k), (0.8, val_k+1),...,(2, val_0)]

    And the latency is '1.33', then this API will locate the values with
    timestamps close to 2 - 1.33 = 0.67. So in this case, it will return the
    pair (0.7, val_k) and (0.6, val_k+1), as well as a blending coefficient
    which is calculated by (0.7 - 0.67) / (0.7 - 0.6) = 0.3. This blending coeff
    can be used to linearly interpolate the returned values, i.e. val_1 * (1 -
    coeff) + val_2 * coeff, if the multiply operator is defined.

    Args:
      latency: The time interval(s) to look backwards in the history buffer from
        the most recent timestamp.

    Returns:
      An array of BufferTuple dataclass.

    Raises:
      ValueError: if the latency is negative.
      BufferError: if the buffer is empty.
    """
    buffer_len = len(self._buffer)
    if buffer_len == 0:
      raise BufferError("The buffer is empty. Have you called 'add'?")

    single_latency = isinstance(latency, (int, float))

    if single_latency:
      if latency < 0:
        raise ValueError("Latency cannot be negative.")
    else:
      if any(value < 0 for value in latency):
        raise ValueError("Latency list contains negative values.")
      if latency != sorted(latency):
        raise ValueError("Invalid unsorted latency list.")

    target_list = [latency] if single_latency else latency
    current_time = self._buffer[-1][TIME_IDX]
    target_list = [current_time - offset for offset in target_list]

    result = self._find_values_at(target_list)
    return result[0] if single_latency else result
