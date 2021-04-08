# Lint as: python3
"""A sensor prototype class.

The concept is explained in: go/minitaur-gym-redesign-1.1
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from typing import Any, Iterable, Optional, Sequence, Text, Tuple, Union

import gin
import gym
import numpy as np

from pybullet_envs.minitaur.robots import robot_base
from pybullet_envs.minitaur.robots import time_ordered_buffer

_ARRAY = Sequence[float]
_FloatOrArray = Union[float, _ARRAY]
_DataTypeList = Iterable[Any]

# For sensor with multiput outputs, key of the main observation in output dict.
MAIN_OBS_KEY = ""

# This allows referencing np.float32 in gin config files. For example:
# lidar_sensor.LidarSensor.dtype = @np.float32
gin.external_configurable(np.float32, module="np")
gin.external_configurable(np.float64, module="np")
gin.external_configurable(np.uint8, module="np")


# Observation blenders take a pair of low/high values. The low/high is measured
# by the latency of the observation. So the low value is actually newer in time
# and high value older. The coeff [0, 1] can be thinked as the distance between
# the low and high value value, with 0 being 100% low value and 1 as 100% high
# value.
def linear_obs_blender(low_value: Any, high_value: Any, coeff: float):
  """Linear interpolation of low/high values based on coefficient value."""
  return low_value * (1 - coeff) + high_value * coeff


def closest_obs_blender(low_value: Any, high_value: Any, coeff: float):
  """Choosing the high or low value based on coefficient value."""
  return low_value if coeff < 0.5 else high_value


def newer_obs_blender(low_value: Any, unused_high_value: Any,
                      unused_coeff: float):
  """Always choosing low value, which is the newer value between low/high."""
  return low_value


def older_obs_blender(unused_low_value: Any, high_value: Any,
                      unused_coeff: float):
  """Always choosing the high value, which is the older value between low/high."""
  return high_value


@gin.configurable
class Sensor(object):
  """A prototype class of sensors."""

  def __init__(
      self,
      name: Text,
      sensor_latency: _FloatOrArray,
      interpolator_fn: Any,
      enable_debug_visualization: bool = False,
  ):
    """A basic constructor of the sensor.

    We do not provide a robot instance during __init__, as robot instances may
    be reloaded/recreated during the simulation.

    Args:
      name: the name of the sensor
      sensor_latency: There are two ways to use this expected sensor latency.
        For both methods, the latency should be in the same unit as the sensor
        data timestamp. 1. As a single float number, the observation will be a
        1D array. For real robots, this should be set to 0.0. 2. As an array of
        floats, the observation will be a 2D array based on how long the history
        need to be. Thus, [0.0, 0.1, 0.2] is a history length of 3. Observations
        are stacked on a new axis appended after existing axes.
      interpolator_fn: Function that controls how to interpolate the two values
        that is returned from the time ordered buffer.
      enable_debug_visualization: Whether to draw debugging visualization.
    """
    self._robot = None
    self._name = name
    # Observation space will be implemented by derived classes.
    self._observation_space = None
    self._sensor_latency = sensor_latency
    self._single_latency = True if isinstance(sensor_latency,
                                              (float, int)) else False
    self._enable_debug_visualization = enable_debug_visualization
    if not self._is_valid_latency():
      raise ValueError("sensor_latency is expected to be a non-negative number "
                       "or a non-empty list of non-negative numbers.")
    self._interpolator_fn = interpolator_fn or newer_obs_blender
    self._axis = -1
    timespan = sensor_latency if self._single_latency else max(sensor_latency)
    self._observation_buffer = time_ordered_buffer.TimeOrderedBuffer(
        max_buffer_timespan=timespan)

  def _is_valid_latency(self):
    if self._single_latency:
      return self._sensor_latency >= 0
    if self._sensor_latency:
      return all(value >= 0 for value in self._sensor_latency)
    return False

  def get_name(self) -> Text:
    return self._name

  @property
  def is_single_latency(self) -> bool:
    return self._single_latency

  @property
  def observation_space(self) -> gym.spaces.Space:
    return self._observation_space

  @property
  def enable_debug_visualization(self):
    return self._enable_debug_visualization

  @enable_debug_visualization.setter
  def enable_debug_visualization(self, enable):
    self._enable_debug_visualization = enable

  def get_observation_datatype(self):
    """Returns the data type for the numpy structured array.

    It is recommended to define a list of tuples: (name, datatype, shape)
    Reference: https://docs.scipy.org/doc/numpy-1.15.0/user/basics.rec.html
    Ex:
      return [('motor_angles', np.float64, (8, ))]  # motor angle sensor
      return [('IMU_x', np.float64), ('IMU_z', np.float64), ] # IMU
    Will be deprecated (b/150818246) in favor of observation_space.

    Returns:
      datatype: a list of data types.
    """
    raise NotImplementedError("Deprecated. Are you using the old robot class?")

  def get_lower_bound(self):
    """Returns the lower bound of the observation.

    Will be deprecated (b/150818246) in favor of observation_space.

    Returns:
      lower_bound: the lower bound of sensor values in np.array format
    """
    raise NotImplementedError("Deprecated. Are you using the old robot class?")

  def get_upper_bound(self):
    """Returns the upper bound of the observation.

    Will be deprecated (b/150818246) in favor of observation_space.

    Returns:
      upper_bound: the upper bound of sensor values in np.array format
    """
    raise NotImplementedError("Deprecated. Are you using the old robot class?")

  def _get_original_observation(self) -> Tuple[float, Any]:
    """Gets the non-modified observation.

    Different from the get_observation, which can pollute and sensor data with
    noise and latency, this method shall return the best effort measurements of
    the sensor. For simulated robots, this will return the clean data. For reals
    robots, just return the measurements as is. All inherited class shall
    implement this method.

    Returns:
      The timestamp and the original sensor measurements.

    Raises:
      NotImplementedError for the base class.

    """
    raise NotImplementedError("Not implemented for base class." "")

  def get_observation(self):
    """Returns the observation data.

    Returns:
      observation: the observed sensor values in np.array format
    """
    obs = self._observation_buffer.get_delayed_value(self._sensor_latency)

    if self._single_latency:
      if isinstance(self._observation_space, gym.spaces.Dict):
        return self._interpolator_fn(obs.value_0, obs.value_1, obs.coeff)
      else:
        return np.asarray(
            self._interpolator_fn(obs.value_0, obs.value_1, obs.coeff))
    else:
      if isinstance(self._observation_space, gym.spaces.Dict):
        # interpolate individual sub observation
        interpolated = [
            self._interpolator_fn(data.value_0, data.value_1, data.coeff)
            for data in obs
        ]

        stacked_per_sub_obs = {}
        for k in interpolated[0]:
          stacked_per_sub_obs[k] = np.stack(
              np.asarray([d[k] for d in interpolated]), axis=self._axis)
        return stacked_per_sub_obs
      else:
        obs = np.asarray([
            self._interpolator_fn(data.value_0, data.value_1, data.coeff)
            for data in obs
        ])
        return np.stack(obs, axis=self._axis)

  def set_robot(self, robot: robot_base.RobotBase):
    """Set a robot instance."""
    self._robot = robot

  def get_robot(self):
    """Returns the robot instance."""
    return self._robot

  def on_reset(self, env):
    """A callback function for the reset event.

    Args:
      env: the environment who invokes this callback function.
    """
    self._env = env
    self._observation_buffer.reset()
    self.on_new_observation()

  def on_step(self, env):
    """A callback function for the control step event.

    Args:
      env: the environment who invokes this callback function.
    """
    pass

  def visualize(self):
    """Visualizes the sensor information."""
    pass

  def on_new_observation(self):
    """A callback for each observation received.

    To be differentiated from on_step, which will be called only once per
    control step (i.e. env.step), this API will be called everytime in the
    substep/action repeat loop, when new observations are expected. Each derived
    sensor class should implement this API by implementing:

    my_obs  = call env/robot api to get the observation
    self._observation_buffer.add(my_obs)
    """
    timestamp, obs = self._get_original_observation()
    if self._enable_debug_visualization:
      self.visualize()
    self._observation_buffer.add(timestamp, obs)

  def on_terminate(self, env):
    """A callback function for the terminate event.

    Args:
      env: the environment who invokes this callback function.
    """
    pass

  def _stack_space(self,
                   space: Union[gym.spaces.Box, gym.spaces.Dict],
                   dtype: np.dtype = None) -> Any:
    """Returns stacked version of observation space.

    This stacks a gym.spaces.Box or gym.spaces.Dict action space based on the
    length of the sensor latency and the axis for stacking specified in the
    sensor. A gym.spaces.Box is just stacked, but a gym.spaces.Dict is
    recursively stacked, preserving its dictionary structure while stacking
    any gym.spaces.Box contained within. For example, the input action space:

        gym.spaces.Dict({
            'space_1': gym.spaces.Box(low=0, high=10, shape=(1,)),
            'space_2': gym.spaces.Dict({
                'space_3': gym.spaces.Box(low=0, high=10, shape=(2,)),
                }),
        }))

    would be converted to the following if sensor latency was [0, 1]:

        gym.spaces.Dict({
            'space_1': gym.spaces.Box(low=0, high=10, shape=(1, 2)),
            'space_2': gym.spaces.Dict({
                'space_3': gym.spaces.Box(low=0, high=10, shape=(2, 2)),
                }),
        }))

    Args:
      space: A gym.spaces.Dict or gym.spaces.Box to be stacked.
      dtype: Datatype for the stacking.

    Returns:
      stacked_space: A stacked version of the action space.
    """
    if self._single_latency:
      return space

    # Allow sensors such as last_action_sensor to override the dtype.
    dtype = dtype or space.dtype

    if isinstance(space, gym.spaces.Box):
      return self._stack_space_box(space, dtype)
    elif isinstance(space, gym.spaces.Dict):
      return self._stack_space_dict(space, dtype)
    else:
      raise ValueError(f"Space {space} is an unsupported type.")

  def _stack_space_box(self, space: gym.spaces.Box,
                       dtype: np.dtype) -> gym.spaces.Box:
    """Returns stacked version of a box observation space.

    This stacks a gym.spaces.Box action space based on the length of the sensor
    latency and the axis for stacking specified in the sensor.

    Args:
      space: A gym.spaces.Box to be stacked.
      dtype: Datatype for the stacking

    Returns:
      stacked_space: A stacked version of the gym.spaces.Box action space.
    """
    length = len(self._sensor_latency)
    stacked_space = gym.spaces.Box(
        low=np.repeat(
            np.expand_dims(space.low, axis=self._axis), length,
            axis=self._axis),
        high=np.repeat(
            np.expand_dims(space.high, axis=self._axis),
            length,
            axis=self._axis),
        dtype=dtype)

    return stacked_space

  def _stack_space_dict(self, space: gym.spaces.Dict,
                        dtype: np.dtype) -> gym.spaces.Dict:
    """Returns stacked version of a dict observation space.

    This stacks a gym.spaces.Dict action space based on the length of the sensor
    latency and the recursive structure of the gym.spaces.Dict itself.

    Args:
      space: A gym.spaces.Dict to be stacked.
      dtype: Datatype for the stacking.

    Returns:
      stacked_space: A stacked version of the dictionary action space.
    """
    return gym.spaces.Dict([
        (k, self._stack_space(v, dtype)) for k, v in space.spaces.items()
    ])

  def _encode_obs_dict_keys(self, obs_dict):
    """Encodes sub obs keys of observation dict or observsation space dict."""
    return {encode_sub_obs_key(self, k): v for k, v in obs_dict.items()}


class BoxSpaceSensor(Sensor):
  """A prototype class of sensors with Box shapes."""

  def __init__(self,
               name: Text,
               shape: Tuple[int, ...],
               lower_bound: _FloatOrArray = -np.pi,
               upper_bound: _FloatOrArray = np.pi,
               dtype=np.float64) -> None:
    """Constructs a box type sensor.

    Will be deprecated (b/150818246) once we switch to gym spaces.
    Args:
      name: the name of the sensor
      shape: the shape of the sensor values
      lower_bound: the lower_bound of sensor value, in float or np.array.
      upper_bound: the upper_bound of sensor value, in float or np.array.
      dtype: data type of sensor value
    """
    super(BoxSpaceSensor, self).__init__(
        name=name, sensor_latency=0.0, interpolator_fn=newer_obs_blender)
    self._shape = shape
    self._dtype = dtype

    if isinstance(lower_bound, float):
      self._lower_bound = np.full(shape, lower_bound, dtype=dtype)
    else:
      self._lower_bound = np.array(lower_bound)

    if isinstance(upper_bound, float):
      self._upper_bound = np.full(shape, upper_bound, dtype=dtype)
    else:
      self._upper_bound = np.array(upper_bound)

  def set_robot(self, robot):
    # Since all old robot class do not inherit from RobotBase, we can enforce
    # the checking here.
    if isinstance(robot, robot_base.RobotBase):
      raise ValueError(
          "Cannot use new robot interface RobotBase with old sensor calss.")
    self._robot = robot

  def get_shape(self) -> Tuple[int, ...]:
    return self._shape

  def get_dimension(self) -> int:
    return len(self._shape)

  def get_dtype(self):
    return self._dtype

  def get_observation_datatype(self) -> _DataTypeList:
    """Returns box-shape data type."""
    return [(self._name, self._dtype, self._shape)]

  def get_lower_bound(self) -> _ARRAY:
    """Returns the computed lower bound."""
    return self._lower_bound

  def get_upper_bound(self) -> _ARRAY:
    """Returns the computed upper bound."""
    return self._upper_bound

  def get_observation(self) -> np.ndarray:
    return np.asarray(self._get_observation(), dtype=self._dtype)

  def _get_original_observation(self) -> Tuple[float, Any]:
    # Maintains compatibility with the new sensor classes."""
    raise NotImplementedError("Not implemented for this class.")

  def on_new_observation(self):
    # Maintains compatibility with the new sensor classes."""
    pass


def encode_sub_obs_key(s: Sensor, sub_obs_name: Optional[Text]):
  """Returns a sub observation key for use in observation dictionary."""
  if sub_obs_name == MAIN_OBS_KEY:
    return s.get_name()
  else:
    return f"{s.get_name()}/{sub_obs_name}"
