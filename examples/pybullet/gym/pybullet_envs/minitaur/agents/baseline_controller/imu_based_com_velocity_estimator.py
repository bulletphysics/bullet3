"""State estimator."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from typing import Any, Sequence

from filterpy import kalman
import gin
import numpy as np

from pybullet_envs.minitaur.agents.baseline_controller import state_estimator
from pybullet_envs.minitaur.agents.baseline_controller import time_based_moving_window_filter
from pybullet_envs.minitaur.envs_v2.sensors import accelerometer_sensor
from pybullet_envs.minitaur.envs_v2.sensors import imu_sensor

_DEFAULT_VELOCITY_FILTER_WINDOW = 0.2
_DEFAULT_GYRO_FILTER_WINDOW = 0.1
_DEFAULT_VELOCITY_CLIPPING = 0.8
_STATE_DIMENSION = 3
_GRAVITY = (0.0, 0.0, -9.8)


@gin.configurable
class IMUBasedCOMVelocityEstimator(state_estimator.StateEstimatorBase):
  """Estimate the CoM velocity using IMU sensors and velocities of stance feet.


  Estimates the com velocity of the robot using IMU data and stance feet
  velocities fused by a Kalman Filter. Kalman Filter assumes the true state x
  follows a linear dynamics: x'=Fx+Bu+w, where x' and x denots the
  current and previous state of the robot, F is the state transition matrix,
  B is the control-input matrix, and w is the noise in the dynamics, assuming to
  follow a zero-mean multivariate normal distribution with covariance Q. It also
  assumes that we can obtain a noisy observation of the state with the model
  z=Hx+v, where z is the observed state, H is the observation matrix, and v is
  a noise model, assuming to follow a zero-mean multivariate normal distribution
  with covariance R. In our case, x is the CoM velocity of the robot,
  F=B=H=eye(3), u=dt * CoM acceleration, which is obtained from accelerometer,
  and the noisy observation z is obtained from the negated average velocities at
  the end-effectors in contact with the ground.

  """

  def __init__(
      self,
      robot: Any,
      use_sensor_interface: bool = True,
      accelerometer_variance=0.1,
      observation_variance=0.1,
      initial_variance=0.1,
      velocity_filter_window: float = _DEFAULT_VELOCITY_FILTER_WINDOW,
      gyroscope_filter_window: float = _DEFAULT_GYRO_FILTER_WINDOW,
      contact_detection_threshold: float = 0.0,
      velocity_clipping: float = _DEFAULT_VELOCITY_CLIPPING,
  ):
    """Initializes the class.

    Args:
      robot: A quadruped robot.
      use_sensor_interface: Whether to use the sensor interface to obtain the
        IMU readings or directly get them from the robot class. Former should
        be used in simulation to enable added latency and noise while latter
        should be used on real robot for better performance.
      accelerometer_variance: The estimated variance in the accelerometer
        readings, used in the Kalman Filter.
      observation_variance: The estimated variance in the observed CoM velocity
        from the stance feet velocities, used in the Kalman Filter.
      initial_variance: The variance of the initial distribution for the
        estimated CoM variance.
      velocity_filter_window: The filtering window (in time) used to smooth the
        estimated CoM velocity.
      gyroscope_filter_window: The filtering window (in time) used to smooth the
        input gyroscope readings.
      contact_detection_threshold: Threshold on the contact sensor readings to
        determine whether the foot is in contact with the ground.
      velocity_clipping: Clipping value for the estimated velocity to prevent
        unrealistically large velocity estimations.
    """
    self._robot = robot
    self._contact_detection_threshold = contact_detection_threshold
    self._velocity_clipping = velocity_clipping
    self._use_sensor_interface = use_sensor_interface

    # Use the accelerometer and gyroscope sensor from the robot
    if self._use_sensor_interface:
      for sensor in self._robot.sensors:
        if isinstance(sensor, accelerometer_sensor.AccelerometerSensor):
          self._accelerometer = sensor
        if isinstance(sensor, imu_sensor.IMUSensor):
          self._gyroscope = sensor
      assert hasattr(self, "_accelerometer") and self._accelerometer is not None
      assert hasattr(self, "_gyroscope") and self._gyroscope is not None

    # x is the underlying CoM velocity we want to estimate, z is the observed
    # CoM velocity from the stance feet velocities, and u is the accelerometer
    # readings.
    self._filter = kalman.KalmanFilter(
        dim_x=_STATE_DIMENSION,
        dim_z=_STATE_DIMENSION,
        dim_u=_STATE_DIMENSION)
    # Initialize the state distribution to be a zero-mean multi-variate normal
    # distribution with initial variance.
    self._filter.x = np.zeros(_STATE_DIMENSION)
    self._initial_variance = initial_variance
    self._filter.P = np.eye(_STATE_DIMENSION) * self._initial_variance
    # Covariance matrix for the control variable.
    self._filter.Q = np.eye(_STATE_DIMENSION) * accelerometer_variance
    # Covariance matrix for the observed states.
    self._filter.R = np.eye(_STATE_DIMENSION) * observation_variance

    # observation function (z=H*x+N(0,R))
    self._filter.H = np.eye(_STATE_DIMENSION)
    # state transition matrix (x'=F*x+B*u+N(0,Q))
    self._filter.F = np.eye(_STATE_DIMENSION)
    # Control transition matrix
    self._filter.B = np.eye(_STATE_DIMENSION)

    self._velocity_filter = time_based_moving_window_filter.TimeBasedMovingWindowFilter(
        velocity_filter_window)
    self._gyroscope_filter = time_based_moving_window_filter.TimeBasedMovingWindowFilter(
        gyroscope_filter_window)

    self.reset(0)

  @property
  def com_velocity_body_yaw_aligned_frame(self) -> Sequence[float]:
    """The base velocity projected in the body yaw aligned inertial frame.

    The body yaw aligned frame is a intertia frame where the z axis coincides
    with the yaw of the robot base and the x and y axis coincides with the world
    frame. It has a zero relative velocity/angular velocity
    to the world frame.

    Returns:
      The com velocity in body yaw aligned frame.
    """
    clipped_velocity = np.clip(self._com_velocity_body_yaw_aligned_frame,
                               -self._velocity_clipping,
                               self._velocity_clipping)

    return clipped_velocity

  def reset(self, current_time):
    del current_time
    self._filter.x = np.zeros(_STATE_DIMENSION)
    self._filter.P = np.eye(_STATE_DIMENSION) * self._initial_variance

    self._com_velocity_body_yaw_aligned_frame = np.zeros(_STATE_DIMENSION)

    self._velocity_filter.reset()
    self._gyroscope_filter.reset()

    # Use None instead of 0 in case of a big gap between reset and first step.
    self._last_timestamp = None

  def update(self, current_time):
    del current_time
    current_timestamp = self._robot.timestamp

    # First time step
    if self._last_timestamp is None:
      delta_time_s = 0.0
    else:
      delta_time_s = current_timestamp - self._last_timestamp
    self._last_timestamp = current_timestamp

    if self._use_sensor_interface:
      sensor_acc = np.array(self._accelerometer.get_observation())
      gyroscope_reading = self._gyroscope.get_observation()
    else:
      sensor_acc = np.array(self._robot.base_acceleration_accelerometer)
      gyroscope_reading = self._robot.base_roll_pitch_yaw

    filtered_gyroscope_reading = self._gyroscope_filter.calculate_average(
        gyroscope_reading, current_timestamp)
    # The yaw angle is not used here because reliably estimating the yaw angle
    # of the robot is in general difficult. This leads to a body yaw aligned
    # inertia frame for the estimated velocity.
    yaw_aligned_base_orientation = self._robot.pybullet_client.getQuaternionFromEuler(
        (filtered_gyroscope_reading[0], filtered_gyroscope_reading[1], 0.0))

    rot_mat = self._robot.pybullet_client.getMatrixFromQuaternion(
        yaw_aligned_base_orientation)
    rot_mat = np.array(rot_mat).reshape((_STATE_DIMENSION, _STATE_DIMENSION))
    calibrated_acc = rot_mat.dot(sensor_acc) + np.array(_GRAVITY)
    self._filter.predict(u=calibrated_acc * delta_time_s)
    observed_velocities = []

    foot_contact = [
        np.linalg.norm(contact_force) > self._contact_detection_threshold
        for contact_force in self._robot.feet_contact_forces()
    ]
    for leg_id in range(4):
      if foot_contact[leg_id]:
        jacobian = self._robot.compute_jacobian_for_one_leg(leg_id)
        # Only pick the jacobian related to joint motors
        # TODO(magicmelon): standardize the process of picking out relevant dofs
        com_dof = self._robot.urdf_loader.com_dof
        jacobian = jacobian[:,
                            com_dof + leg_id * 3:com_dof + (leg_id + 1) * 3]
        joint_velocities = self._robot.motor_velocities[leg_id *
                                                        3:(leg_id + 1) * 3]
        leg_velocity_in_base_frame = jacobian.dot(joint_velocities)
        base_velocity_in_base_frame = -leg_velocity_in_base_frame
        observed_velocities.append(rot_mat.dot(base_velocity_in_base_frame))

    if observed_velocities:
      observed_velocities = np.mean(observed_velocities, axis=0)
      self._filter.update(observed_velocities)

    velocity = self._filter.x.copy()

    self._com_velocity_body_yaw_aligned_frame = self._velocity_filter.calculate_average(
        velocity, current_timestamp)
