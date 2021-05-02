# Lint as: python3
"""Add the new laikago robot."""
import gin

from pybullet_envs.minitaur.robots import laikago_constants
from pybullet_envs.minitaur.robots import quadruped_base
from pybullet_envs.minitaur.robots import robot_urdf_loader


@gin.configurable
class Laikago(quadruped_base.QuadrupedBase):
  """The Laikago class that simulates the quadruped from Unitree."""

  def _pre_load(self):
    """Import the Laikago specific constants.
    """
    self._urdf_loader = robot_urdf_loader.RobotUrdfLoader(
        pybullet_client=self._pybullet_client,
        urdf_path=laikago_constants.URDF_PATH,
        enable_self_collision=True,
        init_base_position=laikago_constants.INIT_POSITION,
        init_base_orientation_quaternion=laikago_constants.INIT_ORIENTATION,
        init_joint_angles=laikago_constants.INIT_JOINT_ANGLES,
        joint_offsets=laikago_constants.JOINT_OFFSETS,
        joint_directions=laikago_constants.JOINT_DIRECTIONS,
        motor_names=laikago_constants.MOTOR_NAMES,
        end_effector_names=laikago_constants.END_EFFECTOR_NAMES,
        user_group=laikago_constants.MOTOR_GROUP,
    )

  @classmethod
  def get_constants(cls):
    del cls
    return laikago_constants
