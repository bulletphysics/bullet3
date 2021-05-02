"""A simple locomotion taskand termination condition."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np

import gin
from pybullet_envs.minitaur.envs_v2.tasks import task_interface
from pybullet_envs.minitaur.envs_v2.tasks import task_utils
from pybullet_envs.minitaur.envs_v2.tasks import terminal_conditions
from pybullet_envs.minitaur.envs_v2.utilities import env_utils_v2 as env_utils


@gin.configurable
class SimpleForwardTask(task_interface.Task):
  """A basic "move forward" task."""

  def __init__(self,
               weight=1.0,
               terminal_condition=terminal_conditions
               .default_terminal_condition_for_minitaur,
               divide_with_dt=False,
               clip_velocity=None,
               energy_penalty_coef=0.0,
               min_com_height=None,
               weight_action_accel=None):
    """Initializes the task.

    Args:
      weight: Float. The scaling factor for the reward.
      terminal_condition: Callable object or function. Determines if the task is
        done.
      divide_with_dt: if True, we divide the velocity reward with dt.
      clip_velocity: if not None, we will clip the velocity with this value.
      energy_penalty_coef: Coefficient for the energy penalty that will be added
        to the reward. 0 by default.
      min_com_height: Minimum height for the center of mass of the robot that
        will be used to terminate the task. This is used to obtain task specific
        gaits and set by the config or gin files based on the task and robot.
      weight_action_accel: if not None, penalize the action acceleration.

    Raises:
      ValueError: The energey coefficient is smaller than zero.
    """
    self._weight = weight
    self._terminal_condition = terminal_condition
    self._last_base_position = None
    self._divide_with_dt = divide_with_dt
    self._clip_velocity = clip_velocity
    self._weight_action_accel = weight_action_accel
    self._action_history_sensor = None
    self._min_com_height = min_com_height
    self._energy_penalty_coef = energy_penalty_coef
    self._env = None
    self._step_count = 0
    if energy_penalty_coef < 0:
      raise ValueError("Energy Penalty Coefficient should be >= 0")

  def __call__(self, env):
    return self.reward(env)

  def reset(self, env):
    self._env = env
    self._last_base_position = env_utils.get_robot_base_position(
        self._env.robot)

    if self._weight_action_accel is not None:
      sensor_name = "LastAction"
      self._action_history_sensor = env.sensor_by_name(sensor_name)

  @property
  def step_count(self):
    return self._step_count

  def update(self, env):
    """Updates the internal state of the task."""
    del env
    self._last_base_position = env_utils.get_robot_base_position(
        self._env.robot)

  def reward(self, env):
    """Get the reward without side effects."""
    del env

    self._step_count += 1
    env = self._env
    current_base_position = env_utils.get_robot_base_position(self._env.robot)
    velocity = current_base_position[0] - self._last_base_position[0]
    if self._divide_with_dt:
      velocity /= env.env_time_step
    if self._clip_velocity is not None:
      limit = float(self._clip_velocity)
      velocity = np.clip(velocity, -limit, limit)

    if self._weight_action_accel is None:
      action_acceleration_penalty = 0.0
    else:
      past_actions = self._action_history_sensor.get_observation().T
      action = past_actions[0]
      prev_action = past_actions[1]
      prev_prev_action = past_actions[2]
      acc = action - 2 * prev_action + prev_prev_action
      action_acceleration_penalty = (
          float(self._weight_action_accel) * np.mean(np.abs(acc)))

    reward = velocity
    reward -= action_acceleration_penalty

    # Energy
    if self._energy_penalty_coef > 0:
      energy_reward = -task_utils.calculate_estimated_energy_consumption(
          self._env.robot.motor_torques, self._env.robot.motor_velocities,
          self._env.sim_time_step, self._env.num_action_repeat)
      reward += energy_reward * self._energy_penalty_coef

    return reward * self._weight

  def done(self, env):
    del env
    position = env_utils.get_robot_base_position(self._env.robot)
    if self._min_com_height and position[2] < self._min_com_height:
      return True
    return self._terminal_condition(self._env)
