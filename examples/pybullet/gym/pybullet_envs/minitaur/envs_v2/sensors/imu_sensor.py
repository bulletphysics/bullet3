# Lint as: python3
"""The on robot sensor classes."""

import enum
from typing import Any, Callable, Iterable, Sequence, Type, Text, Union

import gin
import gym
import numpy as np

from pybullet_envs.minitaur.envs_v2.sensors import sensor
from pybullet_envs.minitaur.envs_v2.utilities import noise_generators


@gin.constants_from_enum
class IMUChannel(enum.Enum):
  ROLL = 1,
  PITCH = 2,
  YAW = 3,
  ROLL_RATE = 4,
  PITCH_RATE = 5,
  YAW_RATE = 6,


@gin.configurable
class IMUSensor(sensor.Sensor):
  """An IMU sensor."""

  def __init__(
      self,
      name: Text = "IMU",
      dtype: Type[Any] = np.float64,
      channels: Sequence[IMUChannel] = None,
      lower_bound: Union[float, Iterable[float]] = None,
      upper_bound: Union[float, Iterable[float]] = None,
      noise_generator: Union[Callable[..., Any],
                             noise_generators.NoiseGenerator] = None,
      sensor_latency: Union[float, Sequence[float]] = 0.0,
  ):
    """Constructs IMUSensor.

    Generates separate IMU value channels as per configuration.

    Args:
      name: the name of the sensor.
      dtype: data type of sensor value.
      channels: value channels wants to subscribe. Must be members of the
        IMUChannel class.
      lower_bound: The lower bounds of the sensor reading.
      upper_bound: The upper bounds of the sensor reading.
      noise_generator: Used to add noise to the readings.
      sensor_latency: There are two ways to use this expected sensor latency.
        For both methods, the latency should be in the same unit as the sensor
        data timestamp. 1. As a single float number, the observation will be a
        1D array. For real robots, this should be set to 0.0. 2. As a array of
        floats, the observation will be a 2D array based on how long the history
        need to be. Thus, [0.0, 0.1, 0.2] is a history length of 3.

    Raises:
      ValueError: If no IMU channel is provided and no bounds for the channels.
    """
    super().__init__(
        name=name,
        sensor_latency=sensor_latency,
        interpolator_fn=sensor.linear_obs_blender)
    if channels is None:
      raise ValueError("IMU channels are not provided.")
    self._channels = channels
    self._num_channels = len(self._channels)
    self._noise_generator = noise_generator
    self._dtype = dtype

    if lower_bound is None or upper_bound is None:
      raise ValueError("Must provides bounds for the IMU readings.")

    if isinstance(lower_bound, (float, int)):
      lower_bound = np.full(self._num_channels, lower_bound, dtype=dtype)
    else:
      lower_bound = np.array(lower_bound, dtype=dtype)

    if len(lower_bound) != self._num_channels:
      raise ValueError("length of sensor lower bound {lower_bound} does not"
                       " match the number of channels.")

    if isinstance(upper_bound, (float, int)):
      upper_bound = np.full(self._num_channels, upper_bound, dtype=dtype)
    else:
      upper_bound = np.array(upper_bound, dtype=dtype)

    if len(upper_bound) != self._num_channels:
      raise ValueError("length of sensor upper bound {upper_bound} does not"
                       " match the number of channels.")

    self._observation_space = self._stack_space(
        gym.spaces.Box(
            low=np.array(lower_bound, dtype=self._dtype),
            high=np.array(upper_bound, dtype=self._dtype),
            dtype=self._dtype))

  def get_channels(self) -> Sequence[IMUChannel]:
    return self._channels

  def get_num_channels(self) -> int:
    return self._num_channels

  def _get_original_observation(self):
    rpy = self._robot.base_roll_pitch_yaw
    observations = np.zeros(self._num_channels)
    for i, channel in enumerate(self._channels):
      if channel == IMUChannel.ROLL:
        observations[i] = rpy[0]
      elif channel == IMUChannel.PITCH:
        observations[i] = rpy[1]
      elif channel == IMUChannel.YAW:
        observations[i] = rpy[2]
      elif channel == IMUChannel.ROLL_RATE:
        observations[i] = self._robot.base_roll_pitch_yaw_rate[0]
      elif channel == IMUChannel.PITCH_RATE:
        observations[i] = self._robot.base_roll_pitch_yaw_rate[1]
      elif channel == IMUChannel.YAW_RATE:
        observations[i] = self._robot.base_roll_pitch_yaw_rate[2]

    return self._robot.timestamp, np.array(observations, dtype=self._dtype)

  def get_observation(self) -> np.ndarray:
    delayed_observation = super().get_observation()
    if self._noise_generator:
      if callable(self._noise_generator):
        return self._noise_generator(delayed_observation)
      else:
        return self._noise_generator.add_noise(delayed_observation)

    return delayed_observation
