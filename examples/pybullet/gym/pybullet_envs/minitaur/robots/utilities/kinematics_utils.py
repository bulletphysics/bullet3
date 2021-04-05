# Lint as: python3
"""The inverse kinematic utilities."""
from typing import Optional, Sequence

import numpy as np

from pybullet_utils import bullet_client

_IDENTITY_ROTATION_QUAT = (0, 0, 0, 1)
_IK_SOLVER_TYPE = 0
_LINK_POS_INDEX = 0


def joint_angles_from_link_positions(
    pybullet_client: bullet_client.BulletClient,
    urdf_id: int,
    link_ids: Sequence[int],
    link_positions: Sequence[Sequence[float]],
    positions_are_in_world_frame: bool = False,
    joint_dof_ids: Optional[Sequence[int]] = None,
) -> np.ndarray:
  """Uses Inverse Kinematics to calculate joint angles.

  Args:
    pybullet_client: The bullet client.
    urdf_id: The unique id returned after loading URDF.
    link_ids: The link ids to compute the IK.
    link_positions: The (x, y, z) of the links in the body or the world frame,
      depending on whether the argument link_position_in_world_frame is true.
    positions_are_in_world_frame: Whether the input link positions are specified
      in the world frame or the robot's base frame.
    joint_dof_ids: The degrees of freedom index of the joints we want to extract
      the angles. This can be different from the joint unique ids. For example,
      a fixed joint will increase the joint unique id but will not increase the
      number of degree of freedoms. The joint dof id can be extracted in
      PyBullet by getJointInfo, which corresponds to the "qIndex" in the
      returned values. If not specified, will return all movable joint angles.

  Returns:
    A list of joint angles.
  """
  if positions_are_in_world_frame:
    world_link_positions = link_positions
  else:
    # The PyBullet inverse Kinematics Calculation depends on the current URDF
    # position/orientation, and we cannot pass them to the API. So we have to
    # always query the current base position/orientation to compute world frame
    # link positions.
    urdf_base_position, urdf_base_orientation = (
        pybullet_client.getBasePositionAndOrientation(urdf_id))
    world_link_positions = []
    for link_position in link_positions:
      world_link_position, _ = pybullet_client.multiplyTransforms(
          urdf_base_position, urdf_base_orientation, link_position,
          _IDENTITY_ROTATION_QUAT)
      world_link_positions.append(world_link_position)

  # Notice that the API expects the link positions in the world frame.
  all_joint_angles = pybullet_client.calculateInverseKinematics2(
      urdf_id, link_ids, world_link_positions, solver=_IK_SOLVER_TYPE)

  # Extract the relevant joint angles.
  if joint_dof_ids is None:
    return np.array(all_joint_angles)

  return np.array([all_joint_angles[i] for i in joint_dof_ids])


def link_position_in_world_frame(
    pybullet_client: bullet_client.BulletClient,
    urdf_id: int,
    link_id: int,
):
  """Computes the link's position in the world frame.

  Args:
    pybullet_client: The bullet client.
    urdf_id: The unique id returned after loading URDF.
    link_id: The link id to calculate its position.

  Returns:
    The position of the link in the world frame.
  """
  return np.array(pybullet_client.getLinkState(urdf_id, link_id)[0])


def link_position_in_base_frame(
    pybullet_client: bullet_client.BulletClient,
    urdf_id: int,
    link_id: int,
    base_link_id: Optional[int] = None,
):
  """Computes the link's local position in the robot frame.

  Args:
    pybullet_client: The bullet client.
    urdf_id: The unique id returned after loading URDF.
    link_id: The link to calculate its relative position.
    base_link_id: The link id of the base. For the kinematics robot, such as
      wheeled_robot_base, three additional joints are added to connect the world
      and the base. The base_link_id is no longer -1, and need to be passed in.


  Returns:
    The relative position of the link.
  """
  if base_link_id is None:
    base_position, base_orientation = (
        pybullet_client.getBasePositionAndOrientation(urdf_id))
  else:
    base_link_state = pybullet_client.getLinkState(urdf_id, base_link_id)
    base_position, base_orientation = base_link_state[0], base_link_state[1]

  inverse_translation, inverse_rotation = pybullet_client.invertTransform(
      base_position, base_orientation)

  link_state = pybullet_client.getLinkState(urdf_id, link_id)
  link_position = link_state[0]
  link_local_position, _ = pybullet_client.multiplyTransforms(
      inverse_translation, inverse_rotation, link_position, (0, 0, 0, 1))

  return np.array(link_local_position)


def compute_jacobian(
    pybullet_client: bullet_client.BulletClient,
    urdf_id: int,
    link_id: int,
    all_joint_positions: Sequence[float],
    additional_translation: Optional[Sequence[float]] = (0, 0, 0),
) -> np.ndarray:
  """Computes the Jacobian matrix for the given point on a link.

  CAVEAT: If during URDF loading process additional rotations are provided, the
  computed Jacobian are also transformed.

  Args:
    pybullet_client: The bullet client.
    urdf_id: The unique id returned after loading URDF.
    link_id: The link id as returned from loadURDF.
    all_joint_positions: all the joint positions of the robot. This should
      include the dummy/kinematic drive joints for the wheeled robot.
    additional_translation: The additional translation of the point in the link
      CoM frame.

  Returns:
    The 3 x N transposed Jacobian matrix. where N is the total DoFs of the
    robot. For a quadruped, the first 6 columns of the matrix corresponds to
    the CoM translation and rotation. The columns corresponds to a leg can be
    extracted with indices [6 + leg_id * 3: 6 + leg_id * 3 + 3].
  """

  zero_vec = [0] * len(all_joint_positions)
  jv, _ = pybullet_client.calculateJacobian(
      urdf_id,
      link_id,
      additional_translation,
      all_joint_positions,
      objVelocities=zero_vec,
      objAccelerations=zero_vec)
  jacobian = np.array(jv)
  assert jacobian.shape[0] == 3
  return jacobian


def rotate_to_base_frame(
    pybullet_client: bullet_client.BulletClient,
    urdf_id: int,
    vector: Sequence[float],
    init_orientation_inv_quat: Optional[Sequence[float]] = (0, 0, 0, 1)
) -> np.ndarray:
  """Rotates the input vector to the base coordinate systems.

  Note: This is different from world frame to base frame transformation, as we
  do not apply any translation here.

  Args:
    pybullet_client: The bullet client.
    urdf_id: The unique id returned after loading URDF.
    vector: Input vector in the world frame.
    init_orientation_inv_quat:

  Returns:
    A rotated vector in the base frame.
  """
  _, base_orientation_quat = (
      pybullet_client.getBasePositionAndOrientation(urdf_id))
  _, base_orientation_quat_from_init = pybullet_client.multiplyTransforms(
      positionA=(0, 0, 0),
      orientationA=init_orientation_inv_quat,
      positionB=(0, 0, 0),
      orientationB=base_orientation_quat)
  _, inverse_base_orientation = pybullet_client.invertTransform(
      [0, 0, 0], base_orientation_quat_from_init)

  # PyBullet transforms requires simple list/tuple or it may crash.
  if isinstance(vector, np.ndarray):
    vector_list = vector.tolist()
  else:
    vector_list = vector

  local_vector, _ = pybullet_client.multiplyTransforms(
      positionA=(0, 0, 0),
      orientationA=inverse_base_orientation,
      positionB=vector_list,
      orientationB=(0, 0, 0, 1),
  )
  return np.array(local_vector)
