# Lint as: python3
"""A sensor that measures the acceleration of the robot base."""

from typing import Any, Callable, Sequence, Type, Text, Union

import gin
import gym
import numpy as np

from pybullet_envs.minitaur.envs_v2.sensors import sensor
from pybullet_envs.minitaur.envs_v2.utilities import noise_generators

_ACCELEROMETER_DIM = 3
_DEFAULT_ACCELEROMETER_LOWER_BOUND = (-1, -1, -1)
_DEFAULT_ACCELEROMETER_UPPER_BOUND = (1, 1, 1)


@gin.configurable
class AccelerometerSensor(sensor.Sensor):
  """An Accelerometer sensor."""

  def __init__(
      self,
      name: Text = "Accelerometer",
      dtype: Type[Any] = np.float64,
      lower_bound: Sequence[float] = _DEFAULT_ACCELEROMETER_LOWER_BOUND,
      upper_bound: Sequence[float] = _DEFAULT_ACCELEROMETER_UPPER_BOUND,
      noise_generator: Union[Callable[..., Any],
                             noise_generators.NoiseGenerator] = None,
      sensor_latency: Union[float, Sequence[float]] = 0.0,
  ):
    """Constructs AccelerometerSensor.

    Generates separate IMU value channels as per configuration.

    Args:
      name: the name of the sensor.
      dtype: data type of sensor value.
      lower_bound: The lower bounds of the sensor reading.
      upper_bound: The upper bounds of the sensor reading.
      noise_generator: Used to add noise to the readings.
      sensor_latency: There are two ways to use this expected sensor latency.
        For both methods, the latency should be in the same unit as the sensor
        data timestamp. 1. As a single float number, the observation will be a
        1D array. For real robots, this should be set to 0.0. 2. As a array of
        floats, the observation will be a 2D array based on how long the history
        need to be. Thus, [0.0, 0.1, 0.2] is a history length of 3.

    """
    super().__init__(
        name=name,
        sensor_latency=sensor_latency,
        interpolator_fn=sensor.linear_obs_blender)

    self._noise_generator = noise_generator
    self._dtype = dtype

    if lower_bound is None or upper_bound is None:
      raise ValueError("Must provides bounds for the Accelerometer readings.")

    if len(lower_bound) != _ACCELEROMETER_DIM or len(
        upper_bound) != _ACCELEROMETER_DIM:
      raise ValueError(
          "Bounds must be {} dimensions.".format(_ACCELEROMETER_DIM))

    lower_bound = np.array(lower_bound, dtype=dtype)
    upper_bound = np.array(upper_bound, dtype=dtype)

    self._observation_space = self._stack_space(
        gym.spaces.Box(
            low=np.array(lower_bound, dtype=self._dtype),
            high=np.array(upper_bound, dtype=self._dtype),
            dtype=self._dtype))

  def _get_original_observation(self):
    return self._robot.timestamp, np.array(
        self._robot.base_acceleration_accelerometer, dtype=self._dtype)

  def get_observation(self) -> np.ndarray:
    delayed_observation = super().get_observation()
    if self._noise_generator:
      if callable(self._noise_generator):
        return self._noise_generator(delayed_observation)
      else:
        return self._noise_generator.add_noise(delayed_observation)
    return delayed_observation
