"""The inverse kinematic utilities."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
import typing

_IDENTITY_ORIENTATION = (0, 0, 0, 1)


def joint_angles_from_link_position(
    robot: typing.Any,
    link_position: typing.Sequence[float],
    link_id: int,
    joint_ids: typing.Sequence[int],
    position_in_world_frame=False,
    base_translation: typing.Sequence[float] = (0, 0, 0),
    base_rotation: typing.Sequence[float] = (0, 0, 0, 1)):
  """Uses Inverse Kinematics to calculate joint angles.

  Args:
    robot: A robot instance.
    link_position: The (x, y, z) of the link in the body or the world frame,
      depending on whether the argument position_in_world_frame is true.
    link_id: The link id as returned from loadURDF.
    joint_ids: The positional index of the joints. This can be different from
      the joint unique ids.
    position_in_world_frame: Whether the input link_position is specified
      in the world frame or the robot's base frame.
    base_translation: Additional base translation.
    base_rotation: Additional base rotation.

  Returns:
    A list of joint angles.
  """
  if not position_in_world_frame:
    # Projects to local frame.
    base_position, base_orientation = robot.GetBasePosition(
    ), robot.GetBaseOrientation()
    base_position, base_orientation = robot.pybullet_client.multiplyTransforms(
        base_position, base_orientation, base_translation, base_rotation)

    # Projects to world space.
    world_link_pos, _ = robot.pybullet_client.multiplyTransforms(
        base_position, base_orientation, link_position, _IDENTITY_ORIENTATION)
  else:
    world_link_pos = link_position

  ik_solver = 0
  all_joint_angles = robot.pybullet_client.calculateInverseKinematics(
      robot.quadruped, link_id, world_link_pos, solver=ik_solver)

  # Extract the relevant joint angles.
  joint_angles = [all_joint_angles[i] for i in joint_ids]
  return joint_angles


def link_position_in_world_frame(
    robot: typing.Any,
    link_id: int,
):
  """Computes the link's position in the world frame.

  Args:
    robot: A robot instance.
    link_id: The link id to calculate its position.

  Returns:
    The position of the link in the world frame.
  """
  return np.array(
      robot.pybullet_client.getLinkState(robot.quadruped, link_id)[0])


def link_position_in_base_frame(
    robot: typing.Any,
    link_id: int,
):
  """Computes the link's local position in the robot frame.

  Args:
    robot: A robot instance.
    link_id: The link to calculate its relative position.

  Returns:
    The relative position of the link.
  """
  base_position, base_orientation = robot.GetBasePosition(
  ), robot.GetBaseOrientation()
  inverse_translation, inverse_rotation = robot.pybullet_client.invertTransform(
      base_position, base_orientation)

  link_state = robot.pybullet_client.getLinkState(robot.quadruped, link_id)
  link_position = link_state[0]
  link_local_position, _ = robot.pybullet_client.multiplyTransforms(
      inverse_translation, inverse_rotation, link_position, (0, 0, 0, 1))

  return np.array(link_local_position)


def compute_jacobian(
    robot: typing.Any,
    link_id: int,
):
  """Computes the Jacobian matrix for the given link.

  Args:
    robot: A robot instance.
    link_id: The link id as returned from loadURDF.

  Returns:
    The 3 x N transposed Jacobian matrix. where N is the total DoFs of the
    robot. For a quadruped, the first 6 columns of the matrix corresponds to
    the CoM translation and rotation. The columns corresponds to a leg can be
    extracted with indices [6 + leg_id * 3: 6 + leg_id * 3 + 3].
  """

  all_joint_angles = [state[0] for state in robot.joint_states]
  zero_vec = [0] * len(all_joint_angles)
  jv, _ = robot.pybullet_client.calculateJacobian(robot.quadruped, link_id,
                                                  (0, 0, 0), all_joint_angles,
                                                  zero_vec, zero_vec)
  jacobian = np.array(jv)
  assert jacobian.shape[0] == 3
  return jacobian
