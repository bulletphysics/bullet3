"""Utilities for safety layers."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import collections
import typing
from pybullet_envs.minitaur.robots import robot_config
from pybullet_envs.minitaur.robots.safety import data_types
from pybullet_envs.minitaur.robots.safety import safety_error


def assert_in_bound(name: typing.Text, value: float, bound: data_types.Bound):
  """Check if the given value is within the provided bound.

  Args:
    name: The name of the value.
    value: Number to be checked.
    bound: Contains the lower and upper bounds. The bound is inclusive.

  Raises:
    safety_error.OutofBoundError: when the value is outside the bound.
  """
  if bound.lower <= value <= bound.upper:
    return
  else:
    raise safety_error.OutOfBoundError("{} is out of bound {} for {}".format(
        value, bound, name))


def convert_to_motor_action(
    robot: typing.Any,
    action: typing.Sequence[float],
    control_mode: robot_config.MotorControlMode,
):
  """Converts the input action to generic MotorAction classes.

  Args:
    robot: An robot instance.
    action: The motor commands sent to the robot.
    control_mode: The motor control mode.

  Returns:
    The list of converted MotorAction instances.
  """
  motor_action_list = []
  if control_mode == robot_config.MotorControlMode.POSITION:
    for motor_id, position in enumerate(action):
      motor_action_list.append(
          data_types.MotorAction(
              timestamp=robot.last_action_time,
              position=position,
              position_gain=robot.GetMotorPositionGains()[motor_id],
              velocity=0,
              velocity_gain=robot.GetMotorVelocityGains()[motor_id],
              torque=0))

  if (control_mode == robot_config.MotorControlMode.TORQUE or
      control_mode == robot_config.MotorControlMode.PWM):
    for motor_id, torque in enumerate(action):
      motor_action_list.append(
          data_types.MotorAction(
              timestamp=robot.last_action_time,
              position=0,
              position_gain=0,
              velocity=0,
              velocity_gain=0,
              torque=torque))

  if control_mode == robot_config.MotorControlMode.HYBRID:
    for motor_id in range(robot.num_motors):
      position_index = (
          motor_id * robot_config.HYBRID_ACTION_DIMENSION +
          robot_config.HybridActionIndex.POSITION.value)
      position_gain_index = (
          motor_id * robot_config.HYBRID_ACTION_DIMENSION +
          robot_config.HybridActionIndex.POSITION_GAIN.value)
      velocity_index = (
          motor_id * robot_config.HYBRID_ACTION_DIMENSION +
          robot_config.HybridActionIndex.VELOCITY.value)
      velocity_gain_index = (
          motor_id * robot_config.HYBRID_ACTION_DIMENSION +
          robot_config.HybridActionIndex.VELOCITY_GAIN.value)
      torque_index = (
          motor_id * robot_config.HYBRID_ACTION_DIMENSION +
          robot_config.HybridActionIndex.TORQUE.value)
      motor_action_list.append(
          data_types.MotorAction(
              timestamp=robot.last_action_time,
              position=action[position_index],
              position_gain=action[position_gain_index],
              velocity=action[velocity_index],
              velocity_gain=action[velocity_gain_index],
              torque=action[torque_index]))

  return motor_action_list


class MovingWindowFilter(object):
  """A stable O(1) moving filter for incoming data streams.

  We implement the Neumaier's algorithm to calculate the moving window average,
  which is numerically stable.

  """

  def __init__(self, window_size: int):
    """Initializes the class.

    Args:
      window_size: The moving window size.
    """
    assert window_size > 0
    self._window_size = window_size
    self._value_deque = collections.deque(maxlen=window_size)
    # The moving window sum.
    self._sum = 0
    # The correction term to compensate numerical precision loss during
    # calculation.
    self._correction = 0

  def _neumaier_sum(self, value: float):
    """Update the moving window sum using Neumaier's algorithm.

    For more details please refer to:
    https://en.wikipedia.org/wiki/Kahan_summation_algorithm#Further_enhancements

    Args:
      value: The new value to be added to the window.
    """

    new_sum = self._sum + value
    if abs(self._sum) >= abs(value):
      # If self._sum is bigger, low-order digits of value are lost.
      self._correction += (self._sum - new_sum) + value
    else:
      # low-order digits of sum are lost
      self._correction += (value - new_sum) + self._sum

    self._sum = new_sum

  def calculate_average(self, new_value: float) -> float:
    """Computes the moving window average in O(1) time.

    Args:
      new_value: The new value to enter the moving window.

    Returns:
      The average of the values in the window.

    """
    deque_len = len(self._value_deque)
    if deque_len < self._value_deque.maxlen:
      pass
    else:
      # The left most value to be subtracted from the moving sum.
      self._neumaier_sum(-self._value_deque[0])

    self._neumaier_sum(new_value)
    self._value_deque.append(new_value)

    return (self._sum + self._correction) / self._window_size
