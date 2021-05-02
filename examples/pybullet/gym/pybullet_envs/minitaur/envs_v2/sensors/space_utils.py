# Lint as: python3
"""Converts a list of sensors to gym space."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from typing import List
import gin
import gym
from gym import spaces
import numpy as np

from pybullet_envs.minitaur.envs_v2.sensors import sensor


class UnsupportedConversionError(NotImplementedError):
  """An exception when the function cannot convert sensors to the gym space."""


class AmbiguousDataTypeError(TypeError):
  """An exception when the function cannot determine the data type."""


@gin.configurable
def convert_sensors_to_gym_space(sensors: List[sensor.Sensor]) -> gym.Space:
  """Convert a list of sensors to the corresponding gym space.

  Args:
    sensors: a list of the current sensors

  Returns:
    space: the converted gym space

  Raises:
    UnsupportedConversionError: raises when the function cannot convert the
      given list of sensors.
  """

  if all([
      isinstance(s, sensor.BoxSpaceSensor) and s.get_dimension() == 1
      for s in sensors
  ]):
    return convert_1d_box_sensors_to_gym_space(sensors)
  raise UnsupportedConversionError('sensors = ' + str(sensors))


@gin.configurable
def convert_1d_box_sensors_to_gym_space(
    sensors: List[sensor.Sensor]) -> gym.Space:
  """Convert a list of 1D BoxSpaceSensors to the corresponding gym space.

  Args:
    sensors: a list of the current sensors

  Returns:
    space: the converted gym space

  Raises:
    UnsupportedConversionError: raises when the function cannot convert the
      given list of sensors.
    AmbiguousDataTypeError: raises when the function cannot determine the
      data types because they are not uniform.
  """
  # Check if all sensors are 1D BoxSpaceSensors
  if not all([
      isinstance(s, sensor.BoxSpaceSensor) and s.get_dimension() == 1
      for s in sensors
  ]):
    raise UnsupportedConversionError('sensors = ' + str(sensors))

  # Check if all sensors have the same data type
  sensor_dtypes = [s.get_dtype() for s in sensors]
  if sensor_dtypes.count(sensor_dtypes[0]) != len(sensor_dtypes):
    raise AmbiguousDataTypeError('sensor datatypes are inhomogeneous')

  lower_bound = np.concatenate([s.get_lower_bound() for s in sensors])
  upper_bound = np.concatenate([s.get_upper_bound() for s in sensors])
  observation_space = spaces.Box(
      np.array(lower_bound), np.array(upper_bound), dtype=np.float32)
  return observation_space


@gin.configurable
def convert_sensors_to_gym_space_dictionary(
    sensors: List[sensor.Sensor]) -> gym.Space:
  """Convert a list of sensors to the corresponding gym space dictionary.

  Args:
    sensors: a list of the current sensors

  Returns:
    space: the converted gym space dictionary

  Raises:
    UnsupportedConversionError: raises when the function cannot convert the
      given list of sensors.
  """
  gym_space_dict = {}
  for s in sensors:
    if isinstance(s, sensor.BoxSpaceSensor):
      gym_space_dict[s.get_name()] = spaces.Box(
          np.array(s.get_lower_bound()),
          np.array(s.get_upper_bound()),
          dtype=np.float32)
    elif isinstance(s, sensor.Sensor):
      if isinstance(s.observation_space, spaces.Box):
        gym_space_dict[s.get_name()] = s.observation_space
      elif isinstance(s.observation_space, spaces.Dict):
        gym_space_dict.update(s.observation_space.spaces)
      else:
        raise UnsupportedConversionError(
            f'Unsupported space type {type(s.observation_space)}, '
            f'must be Box or Dict. sensor = {s}')
    else:
      raise UnsupportedConversionError('sensors = ' + str(sensors))
  return spaces.Dict(gym_space_dict)


def create_constant_action(action_space, action_value=0):
  """Create an uniform value action based on the type of action space."""
  if isinstance(action_space, gym.spaces.Dict):
    # gym.spaces.Dict has a shape of None, so construct action over subspaces.
    return {
        sub_name: create_constant_action(sub_space, action_value)
        for sub_name, sub_space in action_space.spaces.items()
    }
  else:  # Presumably gym.spaces.Box, but in case it is not ...
    return np.full(action_space.shape, action_value)


def action_astype(action, dtype):
  """Transform an action to a different datatype."""
  if isinstance(action, dict):
    return {key: action_astype(value, dtype) for key, value in action.items()}
  else:
    return np.array(action, dtype=dtype)
