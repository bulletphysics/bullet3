"""Validates the motor commands."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import typing
from pybullet_envs.minitaur.robots import robot_config
from pybullet_envs.minitaur.robots.safety import data_types
from pybullet_envs.minitaur.robots.safety import utilities
from pybullet_envs.minitaur.robots.safety.python import moving_window_filter

_DEQUE_SIZE = 200


class MotorActionValidator(object):
  """A safety guard to check motor actions.

  Monitors the commands sent to the motor and detect unsafe behaviors.
  """

  def __init__(
      self,
      motor_id: typing.Any,
      position_bound: data_types.Bound,
      position_gain_bound: data_types.Bound,
      velocity_bound: data_types.Bound,
      velocity_gain_bound: data_types.Bound,
      torque_bound: data_types.Bound,
      timestamp_delta_bound: data_types.Bound,
      delta_position_bound: data_types.Bound,
      average_abs_delta_position_bound: data_types.Bound,
      state_buffer_size: int = _DEQUE_SIZE,
  ):
    """Initializes the class.

    Args:
      motor_id: Unique ID for the motor.
      position_bound: The lower/upper bound of the motor angle.
      position_gain_bound: The lower/upper bound of the motor position gain for
        PD control.
      velocity_bound: The lower/upper bound of the motor speed.
      velocity_gain_bound: The lower/upper bound of the motor velocity gain for
        PD control.
      torque_bound: The lower/upper bound of the measured motor torque.
      timestamp_delta_bound: The range of timestamp difference between two
        consecutively received motor states.
      delta_position_bound: The bound between the current motor position and the
        command position, in position control mode.
      average_abs_delta_position_bound: The bound for average motor position and
        command poisition difference.
      state_buffer_size: The buffer size used to calculate the average.
    """
    assert state_buffer_size > 1
    self._last_motor_state = None

    self._motor_id = motor_id
    self._position_bound = position_bound
    self._position_gain_bound = position_gain_bound
    self._velocity_bound = velocity_bound
    self._velocity_gain_bound = velocity_gain_bound
    self._torque_bound = torque_bound
    self._timestamp_delta_bound = timestamp_delta_bound
    self._delta_position_bound = delta_position_bound
    self._average_abs_delta_position_bound = average_abs_delta_position_bound
    self._abs_delta_position_filter = moving_window_filter.MovingWindowFilter(
        state_buffer_size)

  def on_state(self, new_state: data_types.MotorState):
    """Updates the last motor state.

    Args:
      new_state: The latest motor state.
    """
    self._last_motor_state = new_state

  def on_action(self, new_action: data_types.MotorAction,
                control_mode: robot_config.MotorControlMode):
    """Adds a new motor action and validates it.

    Args:
      new_action: A new action that will be send to the motor.
      control_mode: The motor control mode.

    Raises:
      safety_error.OutOfBoundError: When any of the motor action fields or
      state-action difference is out of bound.
    """

    # We first validate the new state.

    motor_str = "motor {} ".format(self._motor_id)
    if (control_mode == robot_config.MotorControlMode.POSITION or
        control_mode == robot_config.MotorControlMode.HYBRID):
      utilities.assert_in_bound(motor_str + "position", new_action.position,
                                self._position_bound)
      utilities.assert_in_bound(motor_str + "velocity", new_action.velocity,
                                self._velocity_bound)
      utilities.assert_in_bound(motor_str + "position gain",
                                new_action.position_gain,
                                self._position_gain_bound)
      utilities.assert_in_bound(motor_str + "velocity gain",
                                new_action.velocity_gain,
                                self._velocity_gain_bound)

    utilities.assert_in_bound(motor_str + "torque", new_action.torque,
                              self._torque_bound)

    if self._last_motor_state is None:
      return

    delta_time = new_action.timestamp - self._last_motor_state.timestamp
    utilities.assert_in_bound(motor_str + "state-action timestamp difference",
                              delta_time, self._timestamp_delta_bound)

    # To detect the bang-bang type controller behavior.
    if (control_mode == robot_config.MotorControlMode.POSITION or
        control_mode == robot_config.MotorControlMode.HYBRID):
      delta_position = new_action.position - self._last_motor_state.position
      utilities.assert_in_bound(motor_str + "state-action position difference",
                                delta_position, self._delta_position_bound)

      average_abs_delta_position = (
          self._abs_delta_position_filter.CalculateAverage(
              abs(delta_position)))
      utilities.assert_in_bound(
          motor_str + "average state-action position difference",
          average_abs_delta_position, self._average_abs_delta_position_bound)
