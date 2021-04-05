# Lint as: python3
"""Helper class to load and manage the robot URDF file in simulation."""

import collections
from typing import Dict, Text, Tuple

import gin
import numpy as np

from pybullet_utils import bullet_client

# Base link does not have a parent joint. So we just use the string "robot_base"
# for reference. The corresponding link/joint id is always -1 in pybullet.
ROBOT_BASE = "robot_base"


def _sub_dict(joint_name_to_id: Dict[Text, int],
              joint_names: Tuple[Text]) -> Dict[Text, int]:
  sub_dict = collections.OrderedDict()
  if joint_names is None:
    return sub_dict
  for name in joint_names:
    sub_dict[name] = joint_name_to_id[name]
  return sub_dict


def convert_to_urdf_joint_angles(
    robot_space_joint_angles: np.ndarray,
    joint_offsets: np.ndarray,
    joint_directions: np.ndarray,
):
  return robot_space_joint_angles * joint_directions + joint_offsets


def convert_to_robot_joint_angles(
    urdf_space_joint_angles: np.ndarray,
    joint_offsets: np.ndarray,
    joint_directions: np.ndarray,
):
  return (urdf_space_joint_angles - joint_offsets) * joint_directions


@gin.configurable
class RobotUrdfLoader(object):
  """A abstract class to manage the robot urdf in sim."""

  def __init__(
      self,
      pybullet_client: bullet_client.BulletClient,
      urdf_path: Text,
      constrained_base: bool = False,
      enable_self_collision: bool = True,
      init_base_position: Tuple[float] = (0, 0, 0),
      init_base_orientation_quaternion: Tuple[float] = None,
      init_base_orientation_rpy: Tuple[float] = None,
      base_names: Tuple[Text] = None,
      init_joint_angles: Dict[Text, float] = None,
      joint_offsets: Dict[Text, float] = None,
      joint_directions: Dict[Text, int] = None,
      motor_names: Tuple[Text] = None,
      end_effector_names: Tuple[Text] = None,
      user_group: Dict[Text, Tuple[Text]] = None,
  ):
    """Initialize the class.

    Args:
      pybullet_client: A pybullet client.
      urdf_path: The path to the URDF to load.
      constrained_base: Whether to create a FIXED constraint to the base of the
        URDF. Needs to be True for kinematic robots. This allows us to "hang"
        the simulated robot in air, and the hanging point can follow arbitrary
        provided paths.
      enable_self_collision: Determines if the robot can collide with itself.
      init_base_position: The base x, y, z after loading.
      init_base_orientation_quaternion: The base rotation after loading.
      init_base_orientation_rpy: The base rotation after loading, presented in
        roll, pitch, yaw.
      base_names: The base joint names. Used to find additional links that
        belong to the base. Optional because the base might only contain a
        single mesh/block, which always has the id of "-1".
      init_joint_angles: Maps joint name to the desired joint pose after loading
        URDF. If not provided, will use the URDF default. This can be a subset
        of all joints in the URDF. This should be in the robot joint convention,
        which can be different from the URDF convention.
      joint_offsets: The "zero" position of joint angles in the URDF space.
      joint_directions: To convert between robot sdk/control and urdf joint
        convention.
      motor_names: The motor joint names in the URDF. Typically a subset of all
        movable joints/DoFs.
      end_effector_names: A subset of joints specifying the end-effector
        joint(s). For example for legged robots the end effectors are the toe
        joints (if provided). For arms this group includes left and right
        grippers.
      user_group: User defined joint groups. For example for quadrupeds, we may
        want to organize all joints according to which leg they belong to.
    """
    self._pybullet_client = pybullet_client
    self._urdf_path = urdf_path
    self._init_base_position = init_base_position
    if init_base_orientation_quaternion is not None:
      self._init_base_orientation_quaternion = init_base_orientation_quaternion
    else:
      if init_base_orientation_rpy is None:
        raise ValueError("Either init_base_orientation_quaterion "
                         "or init_base_orientation_rpy is required")
      self._init_base_orientation_quaternion = (
          self._pybullet_client.getQuaternionFromEuler(
              init_base_orientation_rpy))
    self._constrained_base = constrained_base
    self._enable_self_collision = enable_self_collision
    self._base_names = base_names
    self._init_joint_angles = init_joint_angles
    self._joint_offsets = joint_offsets
    self._joint_directions = joint_directions
    self._motor_names = motor_names
    self._end_effector_names = end_effector_names
    self._user_group = user_group
    self.load(
        enable_self_collision=enable_self_collision,
        init_base_position=init_base_position,
        init_base_orientation_quaternion=init_base_orientation_quaternion,
        init_joint_angles=init_joint_angles)

  def get_base_id_dict(self, name: Tuple[Text] = None) -> Dict[Text, int]:
    if name is None:
      return self._base_dict
    return _sub_dict(self._base_dict, name)

  def get_joint_id_dict(self, name: Tuple[Text] = None) -> Dict[Text, int]:
    if name is None:
      return self._joint_name_to_id
    return _sub_dict(self._joint_name_to_id, name)

  def get_link_id_dict(self, name: Tuple[Text] = None) -> Dict[Text, int]:
    if name is None:
      return self._link_name_to_id
    return _sub_dict(self._link_name_to_id, name)

  def get_motor_id_dict(self, name: Tuple[Text] = None) -> Dict[Text, int]:
    if name is None:
      return self._motor_dict
    return _sub_dict(self._motor_dict, name)

  def get_joint_direction_dict(self,
                               name: Tuple[Text] = None) -> Dict[Text, int]:
    if name is None:
      return self._joint_directions
    return _sub_dict(self._joint_directions, name)

  def get_joint_offset_dict(self,
                            name: Tuple[Text] = None) -> Dict[Text, float]:
    if name is None:
      return self._joint_offsets
    return _sub_dict(self._joint_offsets, name)

  def get_end_effector_id_dict(self,
                               name: Tuple[Text] = None) -> Dict[Text, int]:
    if name is None:
      return self._end_effector_dict
    return _sub_dict(self._end_effector_dict, name)

  @property
  def robot_id(self):
    """Returns the unique object instance id of this loaded URDF in pybullet.

    Note this is different from all other get_{}_id APIs, which returns the
    joint/link id within this robot instance.

    Returns:
      The object id as returned by loadURDF.
    """
    return self._robot_id

  @property
  def all_joint_info(self):
    return self._all_joint_info

  @property
  def user_dict(self):
    return self._user_dict

  @property
  def motor_names(self):
    return self._motor_names

  @property
  def constrained_base(self):
    return self._constrained_base

  def _build_base_dict(self):
    """Builds the base joints dictionary.

    In pybullet, a link's id within the robot always equal to its parent joint
    id. So this base joint dict functionaly is equivalent to the base link dict.
    The dictionary may only contain {ROBOT_BASE: -1} if self._base_names is
    empty.

    Returns:
      The base link (joint) ordered dictionary.
    """
    base_dict = collections.OrderedDict()
    if self._base_names is None:
      base_dict[ROBOT_BASE] = -1
    else:
      base_dict.update(_sub_dict(self._joint_name_to_id, self._base_names))
    return base_dict

  def _build_user_dict(self):
    """Builds a dictionary using user defined joint groups."""
    user_dict = collections.OrderedDict()
    if self._user_group is None:
      return user_dict
    for group_name, group_joint_names in self._user_group.items():
      user_dict[group_name] = collections.OrderedDict()
      user_dict[group_name].update(
          _sub_dict(self._joint_name_to_id, group_joint_names))
    return user_dict

  def _build_all_joint_dict(self):
    """Extracts all joints from the URDF.

    Finds all joints (fixed or movable) in the URDF and extracts the info. This
    includes actuated joints (i.e. motors), and non-actuated joints, e.g. the
    passive joints in Minitaur's four bar mechanism, and fixed joints connecting
    the toe and the lower legs, etc.

    Returns:
      number of joints, all joint information as returned by pybullet, and the
      joint_name_to_id dictionary.

    """
    num_joints = self._pybullet_client.getNumJoints(self._robot_id)
    all_joint_info = [
        self._pybullet_client.getJointInfo(self._robot_id, i)
        for i in range(num_joints)
    ]

    # Remove the default joint dampings to increase sim fidelity.
    for joint_info in all_joint_info:
      joint_id = joint_info[0]
      self._pybullet_client.changeDynamics(
          joint_id, -1, linearDamping=0, angularDamping=0)

    joint_name_to_id = collections.OrderedDict()
    link_name_to_id = collections.OrderedDict([(ROBOT_BASE, -1)])
    for joint_info in all_joint_info:
      joint_name = joint_info[1].decode("UTF-8")
      joint_id = joint_info[0]
      joint_name_to_id[joint_name] = joint_id
      # Index 12 is the name of the joint's child link, and in PyBullet a child
      # link id is always equal to its parent joint id.
      link_name_to_id[joint_info[12].decode("UTF-8")] = joint_id

    return num_joints, all_joint_info, joint_name_to_id, link_name_to_id

  def load(
      self,
      enable_self_collision: bool = None,
      init_base_position: Tuple[float] = None,
      init_base_orientation_quaternion: Tuple[float] = None,
      init_joint_angles: Dict[Text, float] = None,
  ):
    """Reloads the URDF and rebuilds the dictionaries."""
    if enable_self_collision is None:
      enable_self_collision = self._enable_self_collision
    if init_base_position is None:
      init_base_position = self._init_base_position
    if init_base_orientation_quaternion is None:
      init_base_orientation_quaternion = self._init_base_orientation_quaternion

    self._robot_id = self._load_urdf(enable_self_collision, init_base_position,
                                     init_base_orientation_quaternion)

    (self._num_joints, self._all_joint_info, self._joint_name_to_id,
     self._link_name_to_id) = self._build_all_joint_dict()
    self._base_dict = self._build_base_dict()
    self._motor_dict = _sub_dict(self._joint_name_to_id, self._motor_names)
    self._end_effector_dict = _sub_dict(self._joint_name_to_id,
                                        self._end_effector_names)
    self._user_dict = self._build_user_dict()

    self.reset_base_pose(init_base_position, init_base_orientation_quaternion)
    self.reset_joint_angles(init_joint_angles)

  def _load_urdf(self, enable_self_collision: bool,
                 init_base_position: Tuple[float],
                 init_base_orientation_quaternion: Tuple[float]) -> int:
    """Loads the URDF and returns the pybullet id."""
    try:
      if enable_self_collision:
        return self._pybullet_client.loadURDF(
            self._urdf_path,
            init_base_position,
            init_base_orientation_quaternion,
            useFixedBase=self._constrained_base,
            flags=self._pybullet_client.URDF_USE_SELF_COLLISION)
      else:
        return self._pybullet_client.loadURDF(
            self._urdf_path,
            init_base_position,
            init_base_orientation_quaternion,
            useFixedBase=self._constrained_base,
        )
    except:
      print("!!!!!!!!!!!!!!!!")
      print("Error: cannot find file:")
      print(self._urdf_path)
      import sys
      sys.exit(0)
      
  def reset_joint_angles(self, joint_angles: Dict[Text, float] = None):
    """Resets the joint angles.

    Resets the joint poses. This is instanteneously and will ignore the physics
    (e.g. collisions, inertias, etc). Should only be used during the episode
    reset time. Does not work for real robots (other than changing the
    visualization). This API has no effect if both the input joint_angles and
    the self._init_joint_angles are None.

    Args:
      joint_angles: The joint angles in the robot's joint space.

    Raises:
      AttributeError if the joint directions and joint offsets are not provided
      during init.
    """
    if self._init_joint_angles is None:
      return

    if joint_angles is None:
      joint_angles = self._init_joint_angles

    if self._joint_directions is None or self._joint_offsets is None:
      raise AttributeError(
          "joint directions and joint offsets not provided in __init__")

    for joint_name, angle in joint_angles.items():
      urdf_joint_angle = angle * self._joint_directions[
          joint_name] + self._joint_offsets[joint_name]
      self._pybullet_client.resetJointState(
          self._robot_id,
          self._joint_name_to_id[joint_name],
          urdf_joint_angle,
          targetVelocity=0)

  def reset_base_pose(
      self,
      base_position: Tuple[float] = None,
      base_orientation_quaternion: Tuple[float] = None,
  ):
    """Resets the base position and orientation.

    Instanteneously re-position the base pose of the robot. Does not work for
    real robots except for the visualization.

    Args:
      base_position: Base x, y, z position.
      base_orientation_quaternion: Base rotation.
    """
    if base_position is None:
      base_position = self._init_base_position
    if base_orientation_quaternion is None:
      base_orientation_quaternion = self._init_base_orientation_quaternion
    self._pybullet_client.resetBasePositionAndOrientation(
        self._robot_id, base_position, base_orientation_quaternion)
    self._pybullet_client.resetBaseVelocity(self._robot_id, (0, 0, 0),
                                            (0, 0, 0))

  @property
  def com_dof(self):
    return 0 if self._constrained_base else 6
