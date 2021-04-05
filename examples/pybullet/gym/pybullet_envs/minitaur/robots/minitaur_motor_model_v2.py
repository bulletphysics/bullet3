# Lint as: python3
"""This file implements an accurate motor model."""

from typing import Tuple

import gin
import numpy as np

from pybullet_envs.minitaur.robots import hybrid_motor_model
from pybullet_envs.minitaur.robots import robot_config

VOLTAGE_CLIPPING = 50
# TODO(b/73728631): Clamp the pwm signal instead of the OBSERVED_TORQUE_LIMIT.
OBSERVED_TORQUE_LIMIT = 5.7
MOTOR_VOLTAGE = 16.0
MOTOR_RESISTANCE = 0.186
MOTOR_TORQUE_CONSTANT = 0.0954
MOTOR_VISCOUS_DAMPING = 0
MOTOR_POS_LB = 0.5
MOTOR_POS_UB = 2.5


@gin.configurable
class MinitaurMotorModel(hybrid_motor_model.HybridMotorModel):
  """The accurate motor model, which is based on the physics of DC motors.

  The motor model support two types of control: position control and torque
  control. In position control mode, a desired motor angle is specified, and a
  torque is computed based on the internal motor model. When the torque control
  is specified, a pwm signal in the range of [-1.0, 1.0] is converted to the
  torque.

  The internal motor model takes the following factors into consideration:
  pd gains, viscous friction, back-EMF voltage and current-torque profile.
  """

  def __init__(self,
               num_motors: int,
               voltage_clipping: float = VOLTAGE_CLIPPING,
               observed_torque_limit: float = OBSERVED_TORQUE_LIMIT,
               motor_voltage: float = MOTOR_VOLTAGE,
               motor_resistance: float = MOTOR_RESISTANCE,
               motor_torque_constant: float = MOTOR_TORQUE_CONSTANT,
               motor_viscous_damping: float = MOTOR_VISCOUS_DAMPING,
               motor_pos_lb: float = MOTOR_POS_LB,
               motor_pos_ub: float = MOTOR_POS_UB,
               **kwargs):
    super(MinitaurMotorModel, self).__init__(num_motors, **kwargs)
    self._voltage_clipping = voltage_clipping
    self._observed_torque_limit = observed_torque_limit
    self._voltage = motor_voltage
    self._resistance = motor_resistance
    self._torque_constant = motor_torque_constant
    self._viscous_damping = motor_viscous_damping
    self._motor_pos_lb = motor_pos_lb
    self._motor_pos_ub = motor_pos_ub
    self._current_table = [0, 10, 20, 30, 40, 50, 60]
    self._torque_table = [0, 1, 1.9, 2.45, 3.0, 3.25, 3.5]

  def set_voltage(self, voltage):
    self._voltage = voltage

  def get_voltage(self):
    return self._voltage

  def set_viscous_damping(self, viscous_damping):
    self._viscous_damping = viscous_damping

  def get_viscous_dampling(self):
    return self._viscous_damping

  def get_motor_torques(
      self,
      motor_commands: np.ndarray,
      motor_control_mode=None) -> Tuple[np.ndarray, np.ndarray]:
    """Convert the commands (position control or pwm control) to torque.

    Args:
      motor_commands: The desired motor angle if the motor is in position
        control mode. The pwm signal if the motor is in torque control mode.
      motor_control_mode: A MotorControlMode enum.

    Returns:
      actual_torque: The torque that needs to be applied to the motor.
      observed_torque: The torque observed by the sensor.
    """
    if not motor_control_mode:
      motor_control_mode = self._motor_control_mode

    if (motor_control_mode is robot_config.MotorControlMode.TORQUE) or (
        motor_control_mode is robot_config.MotorControlMode.HYBRID):
      raise ValueError(
          "{} is not a supported motor control mode".format(motor_control_mode))

    motor_angle, motor_velocity = self.get_motor_states()
    _, true_motor_velocity = self.get_motor_states(latency=0)

    kp = self._kp
    kd = self._kd

    pwm = -1 * kp * (motor_angle - motor_commands) - kd * motor_velocity
    pwm = np.clip(pwm, -1.0, 1.0)
    return self._convert_to_torque_from_pwm(pwm, true_motor_velocity)

  def _convert_to_torque_from_pwm(self, pwm: np.ndarray,
                                  true_motor_velocity: np.ndarray):
    """Convert the pwm signal to torque.

    Args:
      pwm: The pulse width modulation.
      true_motor_velocity: The true motor velocity at the current moment. It is
        used to compute the back EMF voltage and the viscous damping.

    Returns:
      actual_torque: The torque that needs to be applied to the motor.
      observed_torque: The torque observed by the sensor.
    """
    observed_torque = np.clip(
        self._torque_constant *
        (np.asarray(pwm) * self._voltage / self._resistance),
        -self._observed_torque_limit, self._observed_torque_limit)
    if (self._torque_lower_limits is not None or
        self._torque_upper_limits is not None):
      observed_torque = np.clip(observed_torque, self._torque_lower_limits,
                                self._torque_upper_limits)

    # Net voltage is clipped at 50V by diodes on the motor controller.
    voltage_net = np.clip(
        np.asarray(pwm) * self._voltage -
        (self._torque_constant + self._viscous_damping) *
        np.asarray(true_motor_velocity), -self._voltage_clipping,
        self._voltage_clipping)
    current = voltage_net / self._resistance
    current_sign = np.sign(current)
    current_magnitude = np.absolute(current)
    # Saturate torque based on empirical current relation.
    actual_torque = np.interp(current_magnitude, self._current_table,
                              self._torque_table)
    actual_torque = np.multiply(current_sign, actual_torque)
    actual_torque = np.multiply(self._strength_ratios, actual_torque)
    if (self._torque_lower_limits is not None or
        self._torque_upper_limits is not None):
      actual_torque = np.clip(actual_torque, self._torque_lower_limits,
                              self._torque_upper_limits)
    return observed_torque, actual_torque
