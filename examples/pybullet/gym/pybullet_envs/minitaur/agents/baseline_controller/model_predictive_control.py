# Lint as: python3
"""Classic model predictive control methods."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import math
from typing import Sequence
import numpy as np

_MAX_ABS_RPY = 0.3
_MAX_ABS_ANGULAR_VELOCITY = math.pi

# The center of mass torque is computed using a simple PD control: tau = -KP *
# delta_rotation - KD * delta_angular_velocity
_TORQUE_KP = 2000
_TORQUE_KD = 150

# For center of mass force, we only need to track position in the z direction
# (i.e. maintain the body height), and speed in x-y plane.
_FORCE_KP = 500
_FORCE_KD = 200


def compute_contact_force_projection_matrix(
    foot_positions_in_com_frame: Sequence[Sequence[float]],
    stance_foot_ids: Sequence[int],
) -> np.ndarray:
  r"""Computes the 6 x 3n matrix to map contact force to com dynamics.

  This is essentially the vectorized rhs of com dynamics equation:
      ma = \sum f
      I\omega_dot = \sum r \cross f

  where the summation if over all feet in contact with ground.

  Caveats: Current we have not taken the com rotation into account as we assume
  the com rotation would be small. Ideally we should rotate the foot_positions
  to a world frame centered at com. Also, since absolute yaw angles are not
  accurate dute to drifting, we should use (roll, pitch, 0) to do the foot
  position projection. This feature will be quite useful for MPC.
  TODO(b/143378213): Fix this.

  Args:
    foot_positions_in_com_frame: the local position of each foot.
    stance_foot_ids: The stance foot to be used to assemble the matrix.

  Returns:
    The contact force projection matrix.

  """
  jacobians = []
  for foot_id in stance_foot_ids:
    jv = np.identity(3)
    foot_position = foot_positions_in_com_frame[foot_id]
    x, y, z = foot_position[:3]
    jw = np.array(((0, -z, y), (z, 0, -x), (-y, x, 0)))
    jacobians.append(np.vstack((jv, jw)))

  return np.hstack(jacobians)


def plan_foot_contact_force(
    mass: float,
    inertia: np.ndarray,
    com_position: np.ndarray,
    com_velocity: np.ndarray,
    com_roll_pitch_yaw: np.ndarray,
    com_angular_velocity: np.ndarray,
    foot_positions_in_com_frame: Sequence[Sequence[float]],
    foot_contact_state: Sequence[bool],
    desired_com_position: np.ndarray,
    desired_com_velocity: np.ndarray,
    desired_com_roll_pitch_yaw: np.ndarray,
    desired_com_angular_velocity: np.ndarray,
):
  """Plan the foot contact forces using robot states.

  TODO(b/143382305): Wrap this interface in a MPC class so we can use other
  planning algorithms.

  Args:
    mass: The total mass of the robot.
    inertia: The diagnal elements [Ixx, Iyy, Izz] of the robot.
    com_position: Center of mass position in world frame. Usually we cannot
      accurrately obtain this without motion capture.
    com_velocity: Center of mass velocity in world frame.
    com_roll_pitch_yaw: Center of mass rotation wrt world frame in euler angles.
    com_angular_velocity: The angular velocity (roll_dot, pitch_dot, yaw_dot).
    foot_positions_in_com_frame: The position of all feet/toe joints in the body
      frame.
    foot_contact_state: Indicates if a foot is in contact with the ground.
    desired_com_position: We usually just care about the body height.
    desired_com_velocity: In world frame.
    desired_com_roll_pitch_yaw: We usually care about roll and pitch, since yaw
      measurement can be unreliable.
    desired_com_angular_velocity: Roll and pitch change rate are usually zero.
      Yaw rate is the turning speed of the robot.

  Returns:
    The desired stance foot contact forces.
  """
  del inertia
  del com_position
  body_height = []
  stance_foot_ids = []
  for foot_id, foot_position in enumerate(foot_positions_in_com_frame):
    if not foot_contact_state[foot_id]:
      continue
    stance_foot_ids.append(foot_id)
    body_height.append(foot_position[2])

  avg_bogy_height = abs(sum(body_height) / len(body_height))

  rpy = com_roll_pitch_yaw
  rpy[:2] = np.clip(rpy[:2], -_MAX_ABS_RPY, _MAX_ABS_RPY)
  rpy_dot = com_angular_velocity
  rpy_dot = np.clip(rpy_dot, -_MAX_ABS_ANGULAR_VELOCITY,
                    _MAX_ABS_ANGULAR_VELOCITY)

  com_torque = -avg_bogy_height * (
      _TORQUE_KP * (rpy - desired_com_roll_pitch_yaw) + _TORQUE_KD * rpy_dot)

  # We don't care about the absolute yaw angle in the low level controller.
  # Instead, we stabialize the angular velocity in the z direction.
  com_torque[2] = -avg_bogy_height * _TORQUE_KD * (
      rpy_dot[2] - desired_com_angular_velocity[2])

  # Track a desired com velocity.
  com_force = -_FORCE_KD * (com_velocity - desired_com_velocity)

  # In the z-direction we also want to track the body height.
  com_force[2] += mass * 9.8 - _FORCE_KP * (
      avg_bogy_height - desired_com_position[2])

  com_force_torque = np.concatenate((com_force, com_torque)).transpose()

  # Map the com force torque to foot contact forces.
  foot_force_to_com = compute_contact_force_projection_matrix(
      foot_positions_in_com_frame, stance_foot_ids)
  all_contact_force = -np.matmul(
      np.linalg.pinv(foot_force_to_com), com_force_torque).transpose()
  contact_force = {}

  for i, foot_id in enumerate(stance_foot_ids):
    contact_force[foot_id] = all_contact_force[3 * i:3 * i + 3]

  return contact_force
