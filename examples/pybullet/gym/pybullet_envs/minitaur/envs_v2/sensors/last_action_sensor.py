# Lint as: python3
"""A sensor that returns the last action(s) sent to the environment."""

from typing import Any, Dict, Sequence, Text, Type, Tuple, Union
import gin
import gym
import numpy as np

from pybullet_envs.minitaur.envs_v2.sensors import sensor
from pybullet_envs.minitaur.envs_v2.sensors import space_utils


@gin.configurable
class LastActionSensor(sensor.Sensor):
  """A sensor that reports the last action taken."""

  def __init__(self,
               name: Text = "LastAction",
               dtype: Type[Any] = np.float64,
               sensor_latency: Union[float, Sequence[float]] = 0):
    """Constructs LastActionSensor.

    We do not provide a robot instance during __init__, as robot instances may
    be reloaded/recreated during the simulation.

    Args:
      name: the name of the sensor
      dtype: data type of sensor value.
      sensor_latency: There are two ways to use this expected sensor latency.
        For both methods, the latency should be in the same unit as the sensor
        data timestamp. 1. As a single float number, the observation will be a
        1D array. For real robots, this should be set to 0.0. 2. As a array of
        floats, the observation will be a 2D array based on how long the history
        need to be. Thus, [0.0, 0.1, 0.2] is a history length of 3.
    """
    super().__init__(name=name,
                     sensor_latency=sensor_latency,
                     # We generally don't interpolate actions.
                     interpolator_fn=sensor.older_obs_blender)

    self._dtype = dtype
    self._env = None

  def on_reset(self, env: gym.Env):
    """From the callback, the sensor remembers the environment.

    Args:
      env: the environment who invokes this callback function.
    """
    # Constructs the observation space using the env's action space.
    self._observation_space = self._stack_space(
        env.action_space, dtype=self._dtype)

    # Call the super class methods to initialize the buffers
    super().on_reset(env)

  def _get_original_observation(
      self) -> Tuple[float, Union[Dict[Text, np.ndarray], np.ndarray]]:
    return self._env.get_time(), space_utils.action_astype(
        self._env.last_action, self._dtype)
