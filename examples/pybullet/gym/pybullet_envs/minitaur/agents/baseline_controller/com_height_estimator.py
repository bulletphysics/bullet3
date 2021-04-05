"""State estimator for robot height."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import copy
from typing import Any, Sequence

import gin

from pybullet_envs.minitaur.agents.baseline_controller import state_estimator


@gin.configurable
class COMHeightEstimator(state_estimator.StateEstimatorBase):
  """Estimate the CoM height using base orientation and local toe positions."""

  def __init__(
      self,
      robot: Any,
      com_estimate_leg_indices: Sequence[int] = (0, 1, 2, 3),
      initial_com_height: float = 0.45,
  ):
    """Initializes the class.

    Args:
      robot: A quadruped robot.
      com_estimate_leg_indices: Leg indices used in estimating the CoM height.
      initial_com_height: CoM height used during reset.
    """
    self._robot = robot
    self._com_estimate_leg_indices = com_estimate_leg_indices
    self._initial_com_estimate_leg_indices = copy.copy(com_estimate_leg_indices)
    self._initial_com_height = initial_com_height
    self.reset(0)

  @property
  def estimated_com_height(self):
    return self._com_height

  def reset(self, current_time):
    del current_time
    self._com_height = self._initial_com_height
    self._com_estimate_leg_indices = copy.copy(
        self._initial_com_estimate_leg_indices)

  def update(self, current_time):
    del current_time
    local_toe_poses = self._robot.foot_positions(
        position_in_world_frame=False)
    # We rotate the local toe positions into the world orientation centered
    # at the robot base to estimate the height of the robot.
    world_toe_poses = []
    for toe_p in local_toe_poses:
      world_toe_poses.append(
          self._robot.pybullet_client.multiplyTransforms(
              positionA=(0, 0, 0),
              orientationA=self._robot.base_orientation_quaternion,
              positionB=toe_p,
              orientationB=(0, 0, 0, 1))[0])
    mean_height = 0.0
    num_toe_in_contact = 0
    for leg_id in self._com_estimate_leg_indices:
      mean_height += world_toe_poses[leg_id][2]
      num_toe_in_contact += 1
    mean_height /= num_toe_in_contact
    self._com_height = abs(mean_height)

  @property
  def com_estimate_leg_indices(self):
    return self._com_estimate_leg_indices

  @com_estimate_leg_indices.setter
  def com_estimate_leg_indices(self, leg_indices):
    self._com_estimate_leg_indices = leg_indices
