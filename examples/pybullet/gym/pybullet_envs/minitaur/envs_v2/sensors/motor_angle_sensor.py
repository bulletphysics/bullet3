# Lint as: python3
"""The on robot sensor classes."""

from typing import Any, Callable, Iterable, Optional, Sequence, Type, Text, Tuple, Union

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
class MotorAngleSensor(sensor.Sensor):
  """A sensor that reads motor angles from the robot."""

  def __init__(self,
               name: Text = "MotorAngle",
               dtype: Type[Any] = np.float64,
               lower_bound: Optional[Union[float, Iterable[float]]] = None,
               upper_bound: Optional[Union[float, Iterable[float]]] = None,
               noise_generator: Union[Callable[..., Any],
                                      noise_generators.NoiseGenerator] = None,
               sensor_latency: Union[float, Sequence[float]] = 0.0,
               observe_sine_cosine: bool = False):
    """Initializes the class.

    Args:
      name: The name of the sensor.
      dtype: The datatype of this sensor.
      lower_bound: The optional lower bounds of the sensor reading. If not
        provided, will extract from the motor limits of the robot class.
      upper_bound: The optional upper bounds of the sensor reading. If not
        provided, will extract from the motor limits of the robot class.
      noise_generator: Adds noise to the sensor readings.
      sensor_latency: There are two ways to use this expected sensor latency.
        For both methods, the latency should be in the same unit as the sensor
        data timestamp. 1. As a single float number, the observation will be a
        1D array. For real robots, this should be set to 0.0. 2. As a array of
        floats, the observation will be a 2D array based on how long the history
        need to be. Thus, [0.0, 0.1, 0.2] is a history length of 3.
       observe_sine_cosine: whether to observe motor angles as sine and cosine
         values.
    """
    super().__init__(
        name=name,
        sensor_latency=sensor_latency,
        interpolator_fn=sensor.linear_obs_blender)
    self._noise_generator = noise_generator
    self._dtype = dtype
    self._lower_bound = lower_bound
    self._upper_bound = upper_bound
    self._observe_sine_cosine = observe_sine_cosine

  def set_robot(self, robot):
    self._robot = robot
    # Creates the observation space based on the robot motor limitations.
    if self._observe_sine_cosine:
      lower_bound = _convert_to_np_array(-1, 2 * self._robot.num_motors)
    elif self._lower_bound:
      lower_bound = _convert_to_np_array(self._lower_bound,
                                         self._robot.num_motors)
    else:
      lower_bound = _convert_to_np_array(
          self._robot.motor_limits.angle_lower_limits, self._robot.num_motors)
    if self._observe_sine_cosine:
      upper_bound = _convert_to_np_array(1, 2 * self._robot.num_motors)
    elif self._upper_bound:
      upper_bound = _convert_to_np_array(self._upper_bound,
                                         self._robot.num_motors)
    else:
      upper_bound = _convert_to_np_array(
          self._robot.motor_limits.angle_upper_limits, self._robot.num_motors)

    self._observation_space = self._stack_space(
        gym.spaces.Box(low=lower_bound, high=upper_bound, dtype=self._dtype))

  def _get_original_observation(self):
    if self._observe_sine_cosine:
      return self._robot.timestamp, np.hstack(
          (np.cos(self._robot.motor_angles), np.sin(self._robot.motor_angles)))
    else:
      return self._robot.timestamp, self._robot.motor_angles

  def get_observation(self) -> np.ndarray:
    delayed_observation = super().get_observation()
    if self._noise_generator:
      if callable(self._noise_generator):
        return self._noise_generator(delayed_observation)
      else:
        return self._noise_generator.add_noise(delayed_observation)

    return delayed_observation
