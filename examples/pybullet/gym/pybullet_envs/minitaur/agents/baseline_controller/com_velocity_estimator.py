"""State estimator."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from typing import Any, Sequence

import gin
import numpy as np

from pybullet_envs.minitaur.agents.baseline_controller import state_estimator
from pybullet_envs.minitaur.robots.safety.python import moving_window_filter

_DEFAULT_WINDOW_SIZE = 20


@gin.configurable
class COMVelocityEstimator(state_estimator.StateEstimatorBase):
  """Estimate the CoM velocity using on board sensors.


  Requires knowledge about the base velocity in world frame, which for example
  can be obtained from a MoCap system. This estimator will filter out the high
  frequency noises in the velocity so the results can be used with controllers
  reliably.

  """

  def __init__(
      self,
      robot: Any,
      window_size: int = _DEFAULT_WINDOW_SIZE,
  ):
    self._robot = robot
    self._window_size = window_size
    self.reset(0)

  @property
  def com_velocity_body_yaw_aligned_frame(self) -> Sequence[float]:
    """The base velocity projected in the body aligned inertial frame.

    The body aligned frame is a intertia frame that coincides with the body
    frame, but has a zero relative velocity/angular velocity to the world frame.

    Returns:
      The com velocity in body aligned frame.
    """
    return self._com_velocity_body_yaw_aligned_frame

  @property
  def com_velocity_world_frame(self) -> Sequence[float]:
    return self._com_velocity_world_frame

  def reset(self, current_time):
    del current_time
    # We use a moving window filter to reduce the noise in velocity estimation.
    self._velocity_filter_x = moving_window_filter.MovingWindowFilter(
        window_size=self._window_size)
    self._velocity_filter_y = moving_window_filter.MovingWindowFilter(
        window_size=self._window_size)
    self._velocity_filter_z = moving_window_filter.MovingWindowFilter(
        window_size=self._window_size)
    self._com_velocity_world_frame = np.array((0, 0, 0))
    self._com_velocity_body_yaw_aligned_frame = np.array((0, 0, 0))

  def update(self, current_time):
    del current_time
    velocity = self._robot.base_velocity

    vx = self._velocity_filter_x.CalculateAverage(velocity[0])
    vy = self._velocity_filter_y.CalculateAverage(velocity[1])
    vz = self._velocity_filter_z.CalculateAverage(velocity[2])
    self._com_velocity_world_frame = np.array((vx, vy, vz))

    base_orientation = self._robot.base_orientation_quaternion
    _, inverse_rotation = self._robot.pybullet_client.invertTransform(
        (0, 0, 0), base_orientation)

    self._com_velocity_body_yaw_aligned_frame, _ = (
        self._robot.pybullet_client.multiplyTransforms(
            (0, 0, 0), inverse_rotation, self._com_velocity_world_frame,
            (0, 0, 0, 1)))
