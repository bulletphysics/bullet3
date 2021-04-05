"""Software safety layer for robot control.

Validates the motor states received from the motor encoder.

"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import typing

from pybullet_envs.minitaur.robots.safety import data_types
from pybullet_envs.minitaur.robots.safety import utilities
from pybullet_envs.minitaur.robots.safety.python import moving_window_filter

# The default internal buffer size for the MotorStateValidator.
_DEQUE_SIZE = 200


class MotorStateValidator(object):
  """A safety guard to check motor states.

  Monitors the status of the motor and detects anomalies in the
  readings. For example, the class will throw safety errors if the motor
  velocity is too large. Currently we support checking of motor angle, velocity,
  gain, torque, as well as the timestamp interval.

  Attributes:
    last_state: The last received motor state.
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
      average_abs_velocity_bound: data_types.Bound,
      average_abs_power_bound: data_types.Bound,
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
      average_abs_velocity_bound: The average absolute velocity limit.
      average_abs_power_bound: The average absolute mechanical power limit.
      state_buffer_size: The buffer size used to calculate the average.
    """
    assert state_buffer_size > 1
    self.last_state = None

    self._motor_id = motor_id
    self._position_bound = position_bound
    self._position_gain_bound = position_gain_bound
    self._velocity_bound = velocity_bound
    self._velocity_gain_bound = velocity_gain_bound
    self._torque_bound = torque_bound
    self._timestamp_delta_bound = timestamp_delta_bound
    self._average_abs_velocity_bound = average_abs_velocity_bound
    self._average_abs_power_bound = average_abs_power_bound

    # For velocity/power, we use a filter to compute their averages
    # over a small period. This is to avoid the noisy readings giving false
    # positive.
    self._abs_velocity_filter = moving_window_filter.MovingWindowFilter(
        state_buffer_size)
    self._abs_power_filter = moving_window_filter.MovingWindowFilter(
        state_buffer_size)

  def on_state(self, new_state: data_types.MotorState):
    """Adds a new motor state and validates it.

    Will validate both the instantenous state as well as statitical
    averages.

    Args:
      new_state: A new state from the motor encoder.

    Raises:
      safety_error.OutOfBoundError: When any of the motor readings (e.g.
      position, torque) is out of bound.
    """

    # We first validate the new state.

    motor_str = "motor {} ".format(self._motor_id)
    utilities.assert_in_bound(motor_str + "position", new_state.position,
                              self._position_bound)
    utilities.assert_in_bound(motor_str + "velocity", new_state.velocity,
                              self._velocity_bound)
    utilities.assert_in_bound(motor_str + "position gain",
                              new_state.position_gain,
                              self._position_gain_bound)
    utilities.assert_in_bound(motor_str + "velocity gain",
                              new_state.velocity_gain,
                              self._velocity_gain_bound)
    utilities.assert_in_bound(motor_str + "torque", new_state.torque,
                              self._torque_bound)

    if not self.last_state:
      self.last_state = new_state
      return

    last_state = self.last_state

    # Check if the time interval between two received states are large.

    delta_time = new_state.timestamp - last_state.timestamp
    utilities.assert_in_bound(motor_str + "timestamp", delta_time,
                              self._timestamp_delta_bound)

    average_abs_velocity = self._abs_velocity_filter.CalculateAverage(
        abs(new_state.velocity))
    utilities.assert_in_bound(motor_str + "average velocity",
                              average_abs_velocity,
                              self._average_abs_velocity_bound)

    average_abs_power = self._abs_power_filter.CalculateAverage(
        abs(new_state.velocity * new_state.torque))
    utilities.assert_in_bound(motor_str + "average power", average_abs_power,
                              self._average_abs_power_bound)

    self.last_state = new_state
