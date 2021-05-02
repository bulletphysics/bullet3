# Lint as: python3
"""Quadruped toe position sensor."""

from typing import Any, Callable, Sequence, Text, Tuple, Type, Union

import gin
import gym
import numpy as np

from pybullet_envs.minitaur.envs_v2.sensors import sensor
from pybullet_envs.minitaur.envs_v2.utilities import noise_generators


def _convert_to_np_array(inputs: Union[float, Tuple[float], np.ndarray], dim):
  """Converts the inputs to a numpy array.

  Args:
    inputs: The input scalar or array.
    dim: The dimension of the converted numpy array.

  Returns:
    The converted numpy array.

  Raises:
    ValueError: If the inputs is an array whose dimension does not match the
    provided dimension.
  """
  outputs = None
  if isinstance(inputs, (tuple, np.ndarray)):
    outputs = np.array(inputs)
  else:
    outputs = np.full(dim, inputs)

  if len(outputs) != dim:
    raise ValueError("The inputs array has a different dimension {}"
                     " than provided, which is {}.".format(len(outputs), dim))

  return outputs


@gin.configurable
class ToePositionSensor(sensor.Sensor):
  """A sensor that outputs the toe positions of attached robots or objects."""

  def __init__(
      self,
      name: Text = "toe_position",
      dtype: Type[Any] = np.float64,
      lower_bound: Union[float, Sequence[float]] = -1.0,
      upper_bound: Union[float, Sequence[float]] = 1.0,
      noise_generator: Union[Callable[..., Any],
                             noise_generators.NoiseGenerator] = None,
      sensor_latency: Union[float, Sequence[float]] = 0.0,
  ):
    """Constructor.

    Args:
      name: Name of the sensor.
      dtype: Data type of sensor value.
      lower_bound: The optional lower bounds of the sensor reading.
      upper_bound: The optional upper bounds of the sensor reading.
      noise_generator: Used to add noise to the readings.
      sensor_latency: Single or multiple latency in seconds. See sensor.Sensor
        docstring for details.
    """
    super().__init__(
        name=name,
        sensor_latency=sensor_latency,
        interpolator_fn=sensor.linear_obs_blender)
    self._dtype = dtype
    self._lower_bound = lower_bound
    self._upper_bound = upper_bound
    self._noise_generator = noise_generator

  def set_robot(self, robot):
    self._robot = robot
    num_legs = len(robot.urdf_loader.get_end_effector_id_dict().values())
    lower_bound = _convert_to_np_array(self._lower_bound, num_legs * 3)

    upper_bound = _convert_to_np_array(self._upper_bound, num_legs * 3)

    self._observation_space = self._stack_space(
        gym.spaces.Box(low=lower_bound, high=upper_bound, dtype=self._dtype))

  def _get_original_observation(self) -> Tuple[float, np.ndarray]:
    """Returns raw observation with timestamp."""
    toe_position = np.array(
        self._robot.foot_positions(), dtype=self._dtype).flatten()

    return self._robot.timestamp, toe_position

  def get_observation(self) -> np.ndarray:
    delayed_observation = super().get_observation()
    if self._noise_generator:
      if callable(self._noise_generator):
        return self._noise_generator(delayed_observation)
      else:
        return self._noise_generator.add_noise(delayed_observation)

    return delayed_observation
