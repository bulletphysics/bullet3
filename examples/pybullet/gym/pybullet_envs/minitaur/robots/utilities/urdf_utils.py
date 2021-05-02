# Lint as: python3
"""Utilities for robot URDF models."""

from typing import Text

# In the return from pybullet.getJointInfo, the name of the link whose parent is
# that joint.
LINK_NAME_INDEX = 12
# Indication that link_name_to_id should return -1. This is a constant because
# different URDFs use different names for their base links.
BASE_LINK = ""


def link_name_to_id(link_name: Text, robot_id: int, pybullet_client) -> int:
  """Returns the pybullet integer link id corresponding to link_name.

  Args:
    link_name: The name of the link from the URDF. If this is BASE_LINK, returns
      -1, the link id of the base according to pybullet convention.
    robot_id: Integer id of the robot to which the link belongs, as returned by
      pybullet.loadURDF().
    pybullet_client: Client in which the robot is loaded.

  Returns:
    Integer id of the link.

  Raises:
    ValueError if the link_name is not found in the robot.
  """
  if link_name == BASE_LINK:
    return -1
  link_name_list = []
  for i in range(pybullet_client.getNumJoints(robot_id)):
    joint_info = pybullet_client.getJointInfo(robot_id, i)
    link_name_i = joint_info[LINK_NAME_INDEX].decode("UTF-8")
    if link_name_i == link_name:
      return i
    link_name_list.append(link_name_i)
  raise ValueError("Link name '{}' not found in URDF. Options are: {}".format(
      link_name, link_name_list))


def set_collision_filter_group_mask(urdf_id: int, group: int, mask: int,
                                    pybullet_client):
  """Sets the collision filter group and mask to the robot.

  TODO(tingnan): Check if this has side effects with self collision flags
  when loading URDF.

  Args:
    urdf_id: The URDF id as returned by the loadURDF.
    group: The collision group the robot is in. By default, all dynamics objects
      in PyBullet use collision group 1.
    mask: The collision bit mask to use. See go/pybullet for details.
    pybullet_client: The bullet client to use.
  """
  # We includes "-1" for the base link of the URDF.
  for link_id in range(-1, pybullet_client.getNumJoints(urdf_id)):
    pybullet_client.setCollisionFilterGroupMask(urdf_id, link_id, group, mask)
