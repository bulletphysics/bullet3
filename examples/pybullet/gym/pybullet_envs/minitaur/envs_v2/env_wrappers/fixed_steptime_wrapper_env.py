"""A wrapper that controls the timing between steps.
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import time
import gin


@gin.configurable
class FixedSteptimeWrapperEnv(object):
  """A wrapped LocomotionGymEnv with timing control between steps."""

  def __init__(self,
               gym_env,
               desired_time_between_steps=None):
    """Initializes the wrapper env.

    Args:
      gym_env: An instance of LocomotionGymEnv.
      desired_time_between_steps: The desired time between steps in seconds.
        If this is None, it is set to the env_time_step of the gym_env.
    """
    self._gym_env = gym_env
    if desired_time_between_steps is None:
      self._desired_time_between_steps = gym_env.env_time_step
    else:
      self._desired_time_between_steps = desired_time_between_steps

    self._last_reset_time = time.time()
    self._last_step_time = time.time()
    self._step_counter = 0

  def __getattr__(self, attr):
    return getattr(self._gym_env, attr)

  def reset(self, initial_motor_angles=None, reset_duration=1.0):
    """Reset the environment.

    This function records the timing of the reset.

    Args:
      initial_motor_angles: Not used.
      reset_duration: Not used.

    Returns:
      The observation of the environment after reset.
    """
    obs = self._gym_env.reset(initial_motor_angles=initial_motor_angles,
                              reset_duration=reset_duration)
    self._last_reset_time = time.time()
    self._last_step_time = time.time()
    self._step_counter = 0
    return obs

  def step(self, action):
    """Steps the wrapped environment.

    Args:
      action: Numpy array. The input action from an NN agent.

    Returns:
      The tuple containing the observation, the reward, the epsiode end
      indicator.

    Raises:
      ValueError if input action is None.
    """
    time_between_steps = time.time() - self._last_step_time
    if time_between_steps < self._desired_time_between_steps:
      time.sleep(self._desired_time_between_steps - time_between_steps)
    self._last_step_time = time.time()
    self._step_counter += 1
    return self._gym_env.step(action)

  @property
  def elapsed_time(self):
    """Returns the elapsed time in seconds."""
    return time.time() - self._last_reset_time

  @property
  def steps_per_second(self):
    """Returns the average number of time steps for 1 second."""
    return self._step_counter / self.elapsed_time

  @property
  def seconds_per_step(self):
    """Returns the average time between steps."""
    return self.elapsed_time / self._step_counter
