"""The generic safety checking interface.

Defines the generic safety checker class that can detect bad motor states, imu
states, self-collisions, unsafe motor commands, unusual temperature reading,
etc. Safety criterions are provided by the robot class.
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import typing
from pybullet_envs.minitaur.robots import robot_config
from pybullet_envs.minitaur.robots.safety import data_types
from pybullet_envs.minitaur.robots.safety import motor_action_validator
from pybullet_envs.minitaur.robots.safety import motor_state_validator
from pybullet_envs.minitaur.robots.safety import utilities

_MOTOR_STATE_BUFFER_SIZE = 500


class SafetyChecker(object):
  """The generic safety checking interface."""

  def __init__(
      self,
      robot: typing.Any,
  ):
    """Initilaizes the class.

    TODO(b/131377892): Implement other state checkings including the
    IMU/temperature/contact force if enabled.

    Args:
      robot: A robot instance such like Minitaur/Laikago/Vision60.
    """
    self._robot = robot

    self._motor_state_validators = []
    self._motor_action_validators = []
    for i in range(robot.num_motors):
      self._motor_state_validators.append(
          motor_state_validator.MotorStateValidator(
              motor_id=i,
              position_bound=robot.safety_config.motor_position_bound[i],
              position_gain_bound=robot.safety_config
              .motor_position_gain_bound[i],
              velocity_bound=robot.safety_config.motor_velocity_bound[i],
              velocity_gain_bound=robot.safety_config
              .motor_velocity_gain_bound[i],
              torque_bound=robot.safety_config.motor_torque_bound[i],
              timestamp_delta_bound=robot.safety_config.timestamp_delta_bound,
              average_abs_velocity_bound=robot.safety_config
              .motor_average_abs_velocity_bound[i],
              average_abs_power_bound=robot.safety_config
              .motor_average_abs_power_bound[i],
              state_buffer_size=_MOTOR_STATE_BUFFER_SIZE,
          ))
      self._motor_action_validators.append(
          motor_action_validator.MotorActionValidator(
              motor_id=i,
              position_bound=robot.safety_config.motor_position_bound[i],
              position_gain_bound=robot.safety_config
              .motor_position_gain_bound[i],
              velocity_bound=robot.safety_config.motor_velocity_bound[i],
              velocity_gain_bound=robot.safety_config
              .motor_velocity_gain_bound[i],
              torque_bound=robot.safety_config.motor_torque_bound[i],
              timestamp_delta_bound=robot.safety_config
              .state_action_timestamp_delta_bound,
              delta_position_bound=robot.safety_config
              .motor_delta_position_bound[i],
              average_abs_delta_position_bound=robot.safety_config
              .motor_average_abs_delta_position_bound[i],
              state_buffer_size=_MOTOR_STATE_BUFFER_SIZE,
          ))

  def check_state(self) -> None:
    """Validates the state of the robot.

    TODO(b/131377892): Implement other state checkings including the
    IMU/temperature/contact force if enabled.

    Raises:
      A safety exception if any state checking (motor/imu/etc) fails.
    """

    for motor_id, state_validator, action_validator in zip(
        range(self._robot.num_motors), self._motor_state_validators,
        self._motor_action_validators):
      motor_state = data_types.MotorState(
          timestamp=self._robot.last_state_time,
          position=self._robot.GetMotorAngles()[motor_id],
          velocity=self._robot.GetMotorVelocities()[motor_id],
          position_gain=self._robot.GetMotorPositionGains()[motor_id],
          velocity_gain=self._robot.GetMotorVelocityGains()[motor_id],
          torque=self._robot.GetMotorTorques()[motor_id],
      )
      state_validator.on_state(motor_state)
      action_validator.on_state(motor_state)

  def check_motor_action(
      self,
      action: typing.Sequence[float],
      control_mode: robot_config.MotorControlMode,
  ) -> None:
    """Validate the action w.r.t to the motor states.

    Args:
      action: The motor commands sent to the robot.
      control_mode: The motor control mode.

    Raises:
      A safety exception if action checking fails.
    """
    motor_action_list = utilities.convert_to_motor_action(
        self._robot, action, control_mode)
    for motor_id, validator in enumerate(self._motor_action_validators):
      validator.on_action(motor_action_list[motor_id], control_mode)
