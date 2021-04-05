"""This file implements the robot specific pose tools."""
import math

import attr
import numpy as np

from pybullet_envs.minitaur.envs_v2.utilities import laikago_pose_utils
from pybullet_envs.minitaur.envs_v2.utilities import mini_cheetah_pose_utils
from pybullet_envs.minitaur.envs_v2.utilities import minitaur_pose_utils
from pybullet_envs.minitaur.robots import laikago
from pybullet_envs.minitaur.robots import laikago_v2
from pybullet_envs.minitaur.robots import mini_cheetah
from pybullet_envs.minitaur.robots import minitaur_v2

_ABDUCTION_ACTION_INDEXES = [0, 3, 6, 9]

# The default values used to give a neutral pose for minitaur.
_MINITAUR_DEFAULT_EXTENSION_POS = math.pi / 2
_MINITAUR_DEFAULT_SWING_POS = 0

_LAIKAGO_NEUTRAL_POSE_HIP_ANGLE = math.pi / 4
_LAIKAGO_NEUTRAL_POSE_KNEE_ANGLE = -math.pi / 2
_LAIKAGO_EXTENSION_CONVERSION_MULTIPLIER = 1.0
_LAIKAGO_SWING_CONVERSION_MULTIPLIER = -1.0

_MINI_CHEETAH_NEUTRAL_POSE_HIP_ANGLE = -math.pi / 4
_MINI_CHEETAH_NEUTRAL_POSE_KNEE_ANGLE = math.pi / 2
_MINI_CHEETAH_EXTENSION_CONVERSION_MULTIPLIER = -1.0
_MINI_CHEETAH_SWING_CONVERSION_MULTIPLIER = 1.0


def get_neutral_motor_angles(robot_class):
  """Return a neutral (standing) pose for a given robot type.

  Args:
    robot_class: This returns the class (not the instance) for the robot.
      Currently it supports minitaur, laikago and mini-cheetah.

  Returns:
    Pose object for the given robot. It's either MinitaurPose, LaikagoPose or
    MiniCheetahPose.

  Raises:
    ValueError: If the given robot_class is different than the supported robots.
  """
  if str(robot_class) in [
      str(minitaur_v2.Minitaur)
  ]:
    init_pose = minitaur_pose_utils.leg_pose_to_motor_angles(
        np.array(
            attr.astuple(
                minitaur_pose_utils.MinitaurPose(
                    swing_angle_0=_MINITAUR_DEFAULT_SWING_POS,
                    swing_angle_1=_MINITAUR_DEFAULT_SWING_POS,
                    swing_angle_2=_MINITAUR_DEFAULT_SWING_POS,
                    swing_angle_3=_MINITAUR_DEFAULT_SWING_POS,
                    extension_angle_0=_MINITAUR_DEFAULT_EXTENSION_POS,
                    extension_angle_1=_MINITAUR_DEFAULT_EXTENSION_POS,
                    extension_angle_2=_MINITAUR_DEFAULT_EXTENSION_POS,
                    extension_angle_3=_MINITAUR_DEFAULT_EXTENSION_POS))))
  elif str(robot_class) in [
      str(laikago.Laikago),
      str(laikago_v2.Laikago),
  ]:
    init_pose = np.array(
        attr.astuple(
            laikago_pose_utils.LaikagoPose(
                abduction_angle_0=0,
                hip_angle_0=_LAIKAGO_NEUTRAL_POSE_HIP_ANGLE,
                knee_angle_0=_LAIKAGO_NEUTRAL_POSE_KNEE_ANGLE,
                abduction_angle_1=0,
                hip_angle_1=_LAIKAGO_NEUTRAL_POSE_HIP_ANGLE,
                knee_angle_1=_LAIKAGO_NEUTRAL_POSE_KNEE_ANGLE,
                abduction_angle_2=0,
                hip_angle_2=_LAIKAGO_NEUTRAL_POSE_HIP_ANGLE,
                knee_angle_2=_LAIKAGO_NEUTRAL_POSE_KNEE_ANGLE,
                abduction_angle_3=0,
                hip_angle_3=_LAIKAGO_NEUTRAL_POSE_HIP_ANGLE,
                knee_angle_3=_LAIKAGO_NEUTRAL_POSE_KNEE_ANGLE)))
  elif str(robot_class) == str(mini_cheetah.MiniCheetah):
    init_pose = np.array(
        attr.astuple(
            mini_cheetah_pose_utils.MiniCheetahPose(
                abduction_angle_0=0,
                hip_angle_0=_MINI_CHEETAH_NEUTRAL_POSE_HIP_ANGLE,
                knee_angle_0=_MINI_CHEETAH_NEUTRAL_POSE_KNEE_ANGLE,
                abduction_angle_1=0,
                hip_angle_1=_MINI_CHEETAH_NEUTRAL_POSE_HIP_ANGLE,
                knee_angle_1=_MINI_CHEETAH_NEUTRAL_POSE_KNEE_ANGLE,
                abduction_angle_2=0,
                hip_angle_2=_MINI_CHEETAH_NEUTRAL_POSE_HIP_ANGLE,
                knee_angle_2=_MINI_CHEETAH_NEUTRAL_POSE_KNEE_ANGLE,
                abduction_angle_3=0,
                hip_angle_3=_MINI_CHEETAH_NEUTRAL_POSE_HIP_ANGLE,
                knee_angle_3=_MINI_CHEETAH_NEUTRAL_POSE_KNEE_ANGLE)))
  else:
    init_pose = robot_class.get_neutral_motor_angles()
  return init_pose


def convert_leg_pose_to_motor_angles(robot_class, leg_poses):
  """Convert swing-extend coordinate space to motor angles for a robot type.

  Args:
    robot_class: This returns the class (not the instance) for the robot.
      Currently it supports minitaur, laikago and mini-cheetah.
    leg_poses: A list of leg poses in [swing,extend] or [abduction, swing,
      extend] space for all 4 legs. The order is [abd_0, swing_0, extend_0,
      abd_1, swing_1, extend_1, ...] or [swing_0, extend_0, swing_1, extend_1,
      ...]. Zero swing and zero extend gives a neutral standing pose for all the
      robots. For minitaur, the conversion is fully accurate, for laikago and
      mini-cheetah the conversion is approximate where swing is reflected to hip
      and extend is reflected to both knee and the hip.

  Returns:
    List of motor positions for the selected robot. The list include 8 or 12
    motor angles depending on the given robot type as an argument. Currently
    laikago and mini-cheetah has motors for abduction which does not exist for
    minitaur robot.

  Raises:
    ValueError: Conversion fails due to wrong inputs.
  """
  default_leg_order = ["front_left", "back_left", "front_right", "back_right"]
  leg_order = default_leg_order
  if len(leg_poses) not in [8, 12]:
    raise ValueError("Dimension of the leg pose provided is not 8 or 12.")
  neutral_motor_angles = get_neutral_motor_angles(robot_class)
  motor_angles = leg_poses
  # If it is a robot with 12 motors but the provided leg pose does not contain
  # abduction, extend the pose to include abduction.
  if len(neutral_motor_angles) == 12 and len(leg_poses) == 8:
    for i in _ABDUCTION_ACTION_INDEXES:
      motor_angles.insert(i, 0)
  # If the robot does not have abduction (minitaur) but the input contains them,
  # ignore the abduction angles for the conversion.
  elif len(neutral_motor_angles) == 8 and len(leg_poses) == 12:
    del leg_poses[::3]
  # Minitaur specific conversion calculations using minitaur-specific safety
  # limits.
  if str(robot_class) in [
            
      str(minitaur_v2.Minitaur)
  ]:
    motor_angles = minitaur_pose_utils.leg_pose_to_motor_angles_with_half_pi_offset_and_safety(
        leg_poses)
  # Laikago and mini-cheetah specific conversion calculations.
  elif str(robot_class) in [
      str(mini_cheetah.MiniCheetah),
      str(laikago.Laikago),
      str(laikago_v2.Laikago),
      
  ]:
    swing_scale = 1.0
    extension_scale = 1.0
    # Laikago specific conversion multipliers.
    if str(robot_class) in [
        str(laikago.Laikago),
        str(laikago_v2.Laikago),
        
    ]:
      swing_scale = _LAIKAGO_SWING_CONVERSION_MULTIPLIER
      extension_scale = _LAIKAGO_EXTENSION_CONVERSION_MULTIPLIER
      leg_order = ["front_right", "front_left", "back_right", "back_left"]
    # Mini-cheetah specific multipliers.
    elif str(robot_class) in [str(mini_cheetah.MiniCheetah)]:
      swing_scale = _MINI_CHEETAH_SWING_CONVERSION_MULTIPLIER
      extension_scale = _MINI_CHEETAH_EXTENSION_CONVERSION_MULTIPLIER
    # In this approximate conversion for mini-cheetah and laikago we set hip
    # angle swing + half of the extend and knee angle to extend as rotation.
    # We also scale swing and extend based on some hand-tuned constants.
    multipliers = np.array([1.0, swing_scale, extension_scale] * 4)
    swing_extend_scaled = leg_poses * multipliers
    # Swing is (swing - half of the extension) due to the geometry of the leg.
    extra_swing = swing_extend_scaled * ([0, 0, -0.5] * 4)
    swing_extend_scaled += np.roll(extra_swing, -1)
    motor_angles = list(swing_extend_scaled)
    motor_angles = neutral_motor_angles + motor_angles
    # Change the order of the legs if it is different for the specific robot.
    if leg_order != default_leg_order:
      leg_order = [default_leg_order.index(leg) for leg in leg_order]
      ordered_motor_angles = []
      for i in leg_order:
        ordered_motor_angles.extend(motor_angles[3 * i:3 * i + 3])
      motor_angles = ordered_motor_angles
  else:
    motor_angles = robot_class.convert_leg_pose_to_motor_angles(leg_poses)

  return motor_angles
