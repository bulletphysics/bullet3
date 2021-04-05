# Lint as: python3
"""A generic PD motor model."""

from typing import Tuple, Union
import gin
import numpy as np

from pybullet_envs.minitaur.robots import robot_config
from pybullet_envs.minitaur.robots import time_ordered_buffer

_DEFAULT_BUFFER_SIZE = 200

_HYBRID_ACTION_LEN = len(robot_config.HybridActionIndex)
_HYBRID_POS_INDEX = robot_config.HybridActionIndex.POSITION.value
_HYBRID_KP_INDEX = robot_config.HybridActionIndex.POSITION_GAIN.value
_HYBRID_VEL_INDEX = robot_config.HybridActionIndex.VELOCITY.value
_HYBRID_KD_INDEX = robot_config.HybridActionIndex.VELOCITY_GAIN.value
_HYBRID_TORQUE_INDEX = robot_config.HybridActionIndex.TORQUE.value


def _convert_to_np_array(inputs: Union[float, Tuple[float], np.ndarray], dim):
  """Converts the inputs to a numpy array.

  Args:
    inputs: The input scalar or array.
    dim: The dimension of the converted numpy array.

  Returns:
    The converted numpy array.

  Raises:
    ValueError: If the inputs is an array whose dimension does not match the
    provied dimension.
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
class HybridMotorModel(object):
  """A simple motor model that supports proportional and derivative control.

    When in POSITION mode, the torque is calculated according to the difference
    between current and desired joint angle, as well as the joint velocity
    differences. For more information about PD control, please refer to:
    https://en.wikipedia.org/wiki/PID_controller.

    The model supports a HYBRID mode in which each motor command can be a tuple
    (desired_motor_angle, position_gain, desired_motor_velocity, velocity_gain,
    torque).
  """

  def __init__(
      self,
      num_motors: int,
      pd_latency: float = 0,
      motor_control_mode=robot_config.MotorControlMode.POSITION,
      kp: Union[float, Tuple[float], np.ndarray] = 60,
      kd: Union[float, Tuple[float], np.ndarray] = 1,
      strength_ratios: Union[float, Tuple[float], np.ndarray] = 1,
      torque_lower_limits: Union[float, Tuple[float], np.ndarray] = None,
      torque_upper_limits: Union[float, Tuple[float], np.ndarray] = None,
  ):
    """Initializes the class.

    Args:
      num_motors: The number of motors for parallel computation.
      pd_latency: Simulates the motor controller's latency in reading motor
        angles and velocities.
      motor_control_mode: Can be POSITION, TORQUE, or HYBRID. In POSITION
        control mode, the PD formula is used to track a desired position and a
        zero desired velocity. In TORQUE control mode, we assume a pass through
        of the provided torques. In HYBRID control mode, the users need to
        provie (desired_position, position_gain, desired_velocity,
        velocity_gain, feedfoward_torque) for each motor.
      kp: The default position gains for motors.
      kd: The default velocity gains for motors.
      strength_ratios: The scaling ratio for motor torque outputs. This can be
        useful for quick debugging when sim-to-real gap is observed in the
        actuator behavior.
      torque_lower_limits: The lower bounds for torque outputs.
      torque_upper_limits: The upper bounds for torque outputs. The output
        torques will be clipped by the lower and upper bounds.

    Raises:
      ValueError: If the number of motors provided is negative or zero.
    """
    if num_motors <= 0:
      raise ValueError(
          "Number of motors must be positive, not {}".format(num_motors))
    self._num_motors = num_motors
    self._zero_array = np.full(num_motors, 0)
    self._pd_latency = pd_latency
    self._hybrid_command_dim = _HYBRID_ACTION_LEN * self._num_motors
    self.set_motor_gains(kp, kd)
    self.set_strength_ratios(strength_ratios)
    self._torque_lower_limits = None
    if torque_lower_limits:
      self._torque_lower_limits = _convert_to_np_array(torque_lower_limits,
                                                       self._num_motors)

    self._torque_upper_limits = None
    if torque_upper_limits:
      self._torque_upper_limits = _convert_to_np_array(torque_upper_limits,
                                                       self._num_motors)
    self._motor_control_mode = motor_control_mode

    # The history buffer is used to simulate the pd latency effect.
    # TODO(b/157786642): remove hacks on duplicate timestep once the sim clock
    # is fixed.
    self._observation_buffer = time_ordered_buffer.TimeOrderedBuffer(
        max_buffer_timespan=pd_latency,
        error_on_duplicate_timestamp=False,
        replace_value_on_duplicate_timestamp=True)

  def set_strength_ratios(
      self,
      strength_ratios: Union[float, Tuple[float], np.ndarray],
  ):
    """Sets the strength of each motor relative to the default value.

    Args:
      strength_ratios: The relative strength of motor output, ranging from [0,
        1] inclusive.
    """
    self._strength_ratios = np.clip(
        _convert_to_np_array(strength_ratios, self._num_motors), 0, 1)

  def set_motor_gains(
      self,
      kp: Union[float, Tuple[float], np.ndarray],
      kd: Union[float, Tuple[float], np.ndarray],
  ):
    """Sets the gains of all motors.

    These gains are PD gains for motor positional control. kp is the
    proportional gain and kd is the derivative gain.

    Args:
      kp: Proportional gain of the motors.
      kd: Derivative gain of the motors.
    """
    self._kp = _convert_to_np_array(kp, self._num_motors)
    self._kd = _convert_to_np_array(kd, self._num_motors)

  def get_motor_gains(self):
    """Get the PD gains of all motors.

    Returns:
      Proportional and derivative gain of the motors.
    """
    return self._kp, self._kd

  def reset(self):
    self._observation_buffer.reset()

  def update(self, timestamp, true_motor_positions: np.ndarray,
             true_motor_velocities: np.ndarray):
    # Push these to the buffer
    self._observation_buffer.add(timestamp,
                                 (true_motor_positions, true_motor_velocities))

  def get_motor_torques(
      self,
      motor_commands: np.ndarray,
      motor_control_mode=None) -> Tuple[np.ndarray, np.ndarray]:
    """Computes the motor torques.

    Args:
      motor_commands: The desired motor angle if the motor is in position
        control mode. The pwm signal if the motor is in torque control mode.
      motor_control_mode: A MotorControlMode enum.

    Returns:
      observed_torque: The torque observed. This emulates the limitations in
      torque measurement, which is generally obtained from current estimations.
      actual_torque: The torque that needs to be applied to the motor.

    Raises:
      NotImplementedError if the motor_control_mode is not supported.

    """
    if not motor_control_mode:
      motor_control_mode = self._motor_control_mode

    motor_torques = None

    if motor_control_mode is robot_config.MotorControlMode.TORQUE:
      motor_torques = motor_commands

    if motor_control_mode is robot_config.MotorControlMode.POSITION:
      motor_torques = self._compute_pd_torques(
          desired_motor_angles=motor_commands,
          kp=self._kp,
          desired_motor_velocities=self._zero_array,
          kd=self._kd)

    if motor_control_mode is robot_config.MotorControlMode.HYBRID:
      motor_torques = self._compute_hybrid_action_torques(motor_commands)

    if motor_torques is None:
      raise ValueError(
          "{} is not a supported motor control mode".format(motor_control_mode))

    # Rescale and clip the motor torques as needed.
    motor_torques = self._strength_ratios * motor_torques
    if (self._torque_lower_limits is not None or
        self._torque_upper_limits is not None):
      motor_torques = np.clip(motor_torques, self._torque_lower_limits,
                              self._torque_upper_limits)

    return motor_torques, motor_torques

  def get_motor_states(self, latency=None):
    """Computes observation of motor angle and velocity under latency."""
    if latency is None:
      latency = self._pd_latency
    buffer = self._observation_buffer.get_delayed_value(latency)
    angle_vel_t0 = buffer.value_0
    angle_vel_t1 = buffer.value_1
    coeff = buffer.coeff

    pos_idx = 0
    motor_angles = angle_vel_t0[pos_idx] * (
        1 - coeff) + coeff * angle_vel_t1[pos_idx]
    vel_idx = 1
    motor_velocities = angle_vel_t0[vel_idx] * (
        1 - coeff) + coeff * angle_vel_t1[vel_idx]
    return motor_angles, motor_velocities

  def _compute_pd_torques(
      self,
      desired_motor_angles: np.ndarray,
      kp: np.ndarray,
      desired_motor_velocities,
      kd: np.ndarray,
  ) -> Tuple[np.ndarray, np.ndarray]:
    """Computes the pd torques.

    Args:
      desired_motor_angles: The motor angles to track.
      kp: The position gains.
      desired_motor_velocities: The motor velocities to track.
      kd: The velocity gains.

    Returns:
      The computed motor torques.
    """
    motor_angles, motor_velocities = self.get_motor_states()
    motor_torques = -kp * (motor_angles - desired_motor_angles) - kd * (
        motor_velocities - desired_motor_velocities)

    return motor_torques

  def _compute_hybrid_action_torques(
      self, motor_commands: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Computes the pd torques in the HYBRID mode."""
    assert len(motor_commands) == self._hybrid_command_dim
    kp = motor_commands[_HYBRID_KP_INDEX::_HYBRID_ACTION_LEN]
    kd = motor_commands[_HYBRID_KD_INDEX::_HYBRID_ACTION_LEN]
    desired_motor_angles = motor_commands[_HYBRID_POS_INDEX::_HYBRID_ACTION_LEN]
    desired_motor_velocities = motor_commands[
        _HYBRID_VEL_INDEX::_HYBRID_ACTION_LEN]
    additional_torques = motor_commands[
        _HYBRID_TORQUE_INDEX::_HYBRID_ACTION_LEN]

    return self._compute_pd_torques(desired_motor_angles, kp,
                                    desired_motor_velocities,
                                    kd) + additional_torques
