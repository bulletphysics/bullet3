# Lint as: python3
"""Pybullet simulation of Minitaur robot."""
import math
from typing import Dict, Tuple, Union, Text

from absl import logging
import gin

from pybullet_envs.minitaur.robots import minitaur_constants
from pybullet_envs.minitaur.robots import quadruped_base
from pybullet_envs.minitaur.robots import robot_config
from pybullet_envs.minitaur.robots import robot_urdf_loader


@gin.configurable
class Minitaur(quadruped_base.QuadrupedBase):
  """Minitaur simulation model in pyBullet."""

  def _pre_load(self):
    try:
      use_constrained_base = gin.query_parameter(
          "robot_urdf_loader.RobotUrdfLoader.constrained_base")
    except ValueError:
      use_constrained_base = False
    if use_constrained_base:
      logging.warn(
          "use_constrained_base is currently not compatible with Minitaur's "
          "leg constraints."
      )

    self._urdf_loader = robot_urdf_loader.RobotUrdfLoader(
        pybullet_client=self._pybullet_client,
        enable_self_collision=True,
        urdf_path=minitaur_constants.MINITAUR_URDF_PATH,
        init_base_position=minitaur_constants.INIT_POSITION,
        init_base_orientation_quaternion=minitaur_constants
        .INIT_ORIENTATION_QUAT,
        init_base_orientation_rpy=minitaur_constants.INIT_ORIENTATION_RPY,
        init_joint_angles=minitaur_constants.INIT_JOINT_ANGLES,
        joint_offsets=minitaur_constants.JOINT_OFFSETS,
        joint_directions=minitaur_constants.JOINT_DIRECTIONS,
        motor_names=minitaur_constants.MOTOR_NAMES,
        end_effector_names=minitaur_constants.END_EFFECTOR_NAMES,
        user_group=minitaur_constants.MOTOR_GROUP,
    )

  def _on_load(self):
    """Add hinge constraint for Minitaur's diamond shaped leg after loading."""
    half_pi = math.pi / 2.0
    knee_angle = -2.1834
    for (leg_id, leg_position) in enumerate(minitaur_constants.LEG_ORDER):
      self._pybullet_client.resetJointState(
          self._urdf_loader.robot_id,
          self._joint_id_dict["motor_" + leg_position + "L_joint"],
          self._motor_directions[2 * leg_id] * half_pi,
          targetVelocity=0)
      self._pybullet_client.resetJointState(
          self._urdf_loader.robot_id,
          self._joint_id_dict["knee_" + leg_position + "L_joint"],
          self._motor_directions[2 * leg_id] * knee_angle,
          targetVelocity=0)
      self._pybullet_client.resetJointState(
          self._urdf_loader.robot_id,
          self._joint_id_dict["motor_" + leg_position + "R_joint"],
          self._motor_directions[2 * leg_id + 1] * half_pi,
          targetVelocity=0)
      self._pybullet_client.resetJointState(
          self._urdf_loader.robot_id,
          self._joint_id_dict["knee_" + leg_position + "R_joint"],
          self._motor_directions[2 * leg_id + 1] * knee_angle,
          targetVelocity=0)

      if leg_id < 2:
        self._pybullet_client.createConstraint(
            self._urdf_loader.robot_id,
            self._joint_id_dict["knee_" + leg_position + "R_joint"],
            self._urdf_loader.robot_id,
            self._joint_id_dict["knee_" + leg_position + "L_joint"],
            self._pybullet_client.JOINT_POINT2POINT, [0, 0, 0],
            minitaur_constants.KNEE_CONSTRAINT_POINT_SHORT,
            minitaur_constants.KNEE_CONSTRAINT_POINT_LONG)
      else:
        self._pybullet_client.createConstraint(
            self._urdf_loader.robot_id,
            self._joint_id_dict["knee_" + leg_position + "R_joint"],
            self._urdf_loader.robot_id,
            self._joint_id_dict["knee_" + leg_position + "L_joint"],
            self._pybullet_client.JOINT_POINT2POINT, [0, 0, 0],
            minitaur_constants.KNEE_CONSTRAINT_POINT_LONG,
            minitaur_constants.KNEE_CONSTRAINT_POINT_SHORT)
    self.receive_observation()

  def _reset_joint_angles(self,
                          joint_angles: Union[Tuple[float], Dict[Text,
                                                                 float]] = None,
                          num_reset_steps: int = 100):
    """Resets joint angles of the robot.

    Note that since Minitaur has additional leg constraints on the end
    effectors, directly setting joint angles will lead to constraint violation.
    Instead, we apply motor commands to move the motors to the desired position.

    Args:
      joint_angles: the desired joint angles to reset to.
      num_reset_steps: number of reset steps.
    """
    if joint_angles is None:
      joint_angles = minitaur_constants.INIT_JOINT_ANGLES
    actions = joint_angles
    if isinstance(joint_angles, dict):
      actions = [
          joint_angles[joint_name]
          for joint_name in minitaur_constants.JOINT_NAMES
      ]

    # TODO(b/157786642): since the simulation clock is not stepped here, this
    # reset behaves slightly different compared to the old robot class.
    for _ in range(num_reset_steps):
      self.apply_action(
          actions, motor_control_mode=robot_config.MotorControlMode.POSITION)
      self._pybullet_client.stepSimulation()
      self.receive_observation()
