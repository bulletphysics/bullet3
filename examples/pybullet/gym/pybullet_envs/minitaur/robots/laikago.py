"""Pybullet simulation of a Laikago robot."""
import math
import os
import re
import gin
import numpy as np
from pybullet_utils import transformations
from pybullet_envs.minitaur.envs_v2.utilities import laikago_pose_utils
from pybullet_envs.minitaur.robots import laikago_constants
from pybullet_envs.minitaur.robots import laikago_motor
from pybullet_envs.minitaur.robots import minitaur
from pybullet_envs.minitaur.robots import robot_config

NUM_MOTORS = 12
NUM_LEGS = 4
MOTOR_NAMES = [
    "FR_hip_motor_2_chassis_joint",
    "FR_upper_leg_2_hip_motor_joint",
    "FR_lower_leg_2_upper_leg_joint",
    "FL_hip_motor_2_chassis_joint",
    "FL_upper_leg_2_hip_motor_joint",
    "FL_lower_leg_2_upper_leg_joint",
    "RR_hip_motor_2_chassis_joint",
    "RR_upper_leg_2_hip_motor_joint",
    "RR_lower_leg_2_upper_leg_joint",
    "RL_hip_motor_2_chassis_joint",
    "RL_upper_leg_2_hip_motor_joint",
    "RL_lower_leg_2_upper_leg_joint",
]
INIT_RACK_POSITION = [0, 0, 1]
INIT_POSITION = [0, 0, 0.48]
JOINT_DIRECTIONS = np.array([-1, 1, 1, 1, 1, 1, -1, 1, 1, 1, 1, 1])
HIP_JOINT_OFFSET = 0.0
UPPER_LEG_JOINT_OFFSET = -0.6
KNEE_JOINT_OFFSET = 0.66
DOFS_PER_LEG = 3
JOINT_OFFSETS = np.array(
    [HIP_JOINT_OFFSET, UPPER_LEG_JOINT_OFFSET, KNEE_JOINT_OFFSET] * 4)
PI = math.pi

MAX_MOTOR_ANGLE_CHANGE_PER_STEP = 0.12
_DEFAULT_HIP_POSITIONS = (
    (0.21, -0.1157, 0),
    (0.21, 0.1157, 0),
    (-0.21, -0.1157, 0),
    (-0.21, 0.1157, 0),
)

# Bases on the readings from Laikago's default pose.
INIT_MOTOR_ANGLES = np.array([
    laikago_pose_utils.LAIKAGO_DEFAULT_ABDUCTION_ANGLE,
    laikago_pose_utils.LAIKAGO_DEFAULT_HIP_ANGLE,
    laikago_pose_utils.LAIKAGO_DEFAULT_KNEE_ANGLE
] * NUM_LEGS)

CHASSIS_NAME_PATTERN = re.compile(r"\w+_chassis_\w+")
MOTOR_NAME_PATTERN = re.compile(r"\w+_hip_motor_\w+")
KNEE_NAME_PATTERN = re.compile(r"\w+_lower_leg_\w+")
TOE_NAME_PATTERN = re.compile(r"jtoe\d*")

URDF_NO_TOES = "laikago.urdf"
URDF_WITH_TOES = "laikago_toes_zup.urdf"

_BODY_B_FIELD_NUMBER = 2
_LINK_A_FIELD_NUMBER = 3


@gin.configurable
class Laikago(minitaur.Minitaur):
  """A simulation for the Laikago robot."""

  def __init__(self, urdf_filename=URDF_WITH_TOES, **kwargs):
    self._urdf_filename = urdf_filename
    if "motor_kp" not in kwargs:
      kwargs["motor_kp"] = 100.0
    if "motor_kd" not in kwargs:
      kwargs["motor_kd"] = 2.0
    if "motor_torque_limits" not in kwargs:
      kwargs["motor_torque_limits"] = None

    # enable_clip_motor_commands: Boolean indicating if clipping should be
    # applied to motor commands, which limits the amount of change in joint
    # pose between timesteps.
    if "enable_clip_motor_commands" in kwargs:
      self._enable_clip_motor_commands = kwargs["enable_clip_motor_commands"]
      del kwargs["enable_clip_motor_commands"]
    else:
      self._enable_clip_motor_commands = False

    # The follwing parameters are fixed for the Laikago robot.
    kwargs["num_motors"] = NUM_MOTORS
    kwargs["dofs_per_leg"] = DOFS_PER_LEG
    kwargs["motor_direction"] = JOINT_DIRECTIONS
    kwargs["motor_offset"] = JOINT_OFFSETS
    kwargs["motor_overheat_protection"] = False
    kwargs["motor_model_class"] = laikago_motor.LaikagoMotorModel
    kwargs["safety_config"] = None

    super(Laikago, self).__init__(**kwargs)

  def _LoadRobotURDF(self):
    laikago_urdf_path = self.GetURDFFile()
    if self._self_collision_enabled:
      self.quadruped = self._pybullet_client.loadURDF(
          laikago_urdf_path,
          self._GetDefaultInitPosition(),
          self._GetDefaultInitOrientation(),
          flags=self._pybullet_client.URDF_USE_SELF_COLLISION)
    else:
      self.quadruped = self._pybullet_client.loadURDF(
          laikago_urdf_path, self._GetDefaultInitPosition(),
          self._GetDefaultInitOrientation())

  def _SettleDownForReset(self, default_motor_angles, reset_time):
    self.ReceiveObservation()

    if reset_time <= 0:
      return

    for _ in range(500):
      self._StepInternal(
          INIT_MOTOR_ANGLES,
          motor_control_mode=robot_config.MotorControlMode.POSITION)
    if default_motor_angles is not None:
      num_steps_to_reset = int(reset_time / self.time_step)
      for _ in range(num_steps_to_reset):
        self._StepInternal(
            default_motor_angles,
            motor_control_mode=robot_config.MotorControlMode.POSITION)

  def GetHipPositionsInBaseFrame(self):
    return _DEFAULT_HIP_POSITIONS

  def GetFootContacts(self):
    all_contacts = self._pybullet_client.getContactPoints(bodyA=self.quadruped)

    contacts = [False, False, False, False]
    for contact in all_contacts:
      # Ignore self contacts
      if contact[_BODY_B_FIELD_NUMBER] == self.quadruped:
        continue
      try:
        toe_link_index = self._foot_link_ids.index(
            contact[_LINK_A_FIELD_NUMBER])
        contacts[toe_link_index] = True
      except ValueError:
        continue
    return contacts

  def ComputeJacobian(self, leg_id):
    """Compute the Jacobian for a given leg."""
    # Because of the default rotation in the Laikago URDF, we need to reorder
    # the rows in the Jacobian matrix.
    if self._urdf_filename == URDF_WITH_TOES:
      return super(Laikago, self).ComputeJacobian(leg_id)
    else:
      return super(Laikago, self).ComputeJacobian(leg_id)[(2, 0, 1), :]

  def ResetPose(self, add_constraint):
    del add_constraint
    for name in self._joint_name_to_id:
      joint_id = self._joint_name_to_id[name]
      self._pybullet_client.setJointMotorControl2(
          bodyIndex=self.quadruped,
          jointIndex=(joint_id),
          controlMode=self._pybullet_client.VELOCITY_CONTROL,
          targetVelocity=0,
          force=0)
    for name, i in zip(MOTOR_NAMES, range(len(MOTOR_NAMES))):
      if "hip_motor_2_chassis_joint" in name:
        angle = INIT_MOTOR_ANGLES[i] + HIP_JOINT_OFFSET
      elif "upper_leg_2_hip_motor_joint" in name:
        angle = INIT_MOTOR_ANGLES[i] + UPPER_LEG_JOINT_OFFSET
      elif "lower_leg_2_upper_leg_joint" in name:
        angle = INIT_MOTOR_ANGLES[i] + KNEE_JOINT_OFFSET
      else:
        raise ValueError("The name %s is not recognized as a motor joint." %
                         name)
      self._pybullet_client.resetJointState(
          self.quadruped, self._joint_name_to_id[name], angle, targetVelocity=0)

  def GetURDFFile(self):
    return os.path.join(self._urdf_root, "laikago/" + self._urdf_filename)

  def _BuildUrdfIds(self):
    """Build the link Ids from its name in the URDF file.

    Raises:
      ValueError: Unknown category of the joint name.
    """
    num_joints = self._pybullet_client.getNumJoints(self.quadruped)
    self._chassis_link_ids = [-1]
    self._leg_link_ids = []
    self._motor_link_ids = []
    self._knee_link_ids = []
    self._foot_link_ids = []

    for i in range(num_joints):
      joint_info = self._pybullet_client.getJointInfo(self.quadruped, i)
      joint_name = joint_info[1].decode("UTF-8")
      joint_id = self._joint_name_to_id[joint_name]
      if CHASSIS_NAME_PATTERN.match(joint_name):
        self._chassis_link_ids.append(joint_id)
      elif MOTOR_NAME_PATTERN.match(joint_name):
        self._motor_link_ids.append(joint_id)
      # We either treat the lower leg or the toe as the foot link, depending on
      # the urdf version used.
      elif KNEE_NAME_PATTERN.match(joint_name):
        self._knee_link_ids.append(joint_id)
      elif TOE_NAME_PATTERN.match(joint_name):
        assert self._urdf_filename == URDF_WITH_TOES
        self._foot_link_ids.append(joint_id)
      else:
        raise ValueError("Unknown category of joint %s" % joint_name)

    self._leg_link_ids.extend(self._knee_link_ids)
    self._leg_link_ids.extend(self._foot_link_ids)

    if self._urdf_filename == URDF_NO_TOES:
      self._foot_link_ids.extend(self._knee_link_ids)

    assert len(self._foot_link_ids) == NUM_LEGS
    self._chassis_link_ids.sort()
    self._motor_link_ids.sort()
    self._knee_link_ids.sort()
    self._foot_link_ids.sort()
    self._leg_link_ids.sort()

    return

  def _GetMotorNames(self):
    return MOTOR_NAMES

  def _GetDefaultInitPosition(self):
    if self._on_rack:
      return INIT_RACK_POSITION
    else:
      return INIT_POSITION

  def _GetDefaultInitOrientation(self):
    # The Laikago URDF assumes the initial pose of heading towards z axis,
    # and belly towards y axis. The following transformation is to transform
    # the Laikago initial orientation to our commonly used orientation: heading
    # towards -x direction, and z axis is the up direction.
    if self._urdf_filename == URDF_WITH_TOES:
      return [0, 0, 0, 1]
    else:
      return transformations.quaternion_from_euler(
          ai=math.pi / 2.0, aj=0, ak=math.pi / 2.0, axes="sxyz")

  def GetDefaultInitPosition(self):
    """Get default initial base position."""
    return self._GetDefaultInitPosition()

  def GetDefaultInitOrientation(self):
    """Get default initial base orientation."""
    return self._GetDefaultInitOrientation()

  def GetDefaultInitJointPose(self):
    """Get default initial joint pose."""
    joint_pose = (INIT_MOTOR_ANGLES + JOINT_OFFSETS) * JOINT_DIRECTIONS
    return joint_pose

  def ApplyAction(self, motor_commands, motor_control_mode=None):
    """Clips and then apply the motor commands using the motor model.

    Args:
      motor_commands: np.array. Can be motor angles, torques, hybrid commands,
        or motor pwms (for Minitaur only).N
      motor_control_mode: A MotorControlMode enum.
    """
    if self._enable_clip_motor_commands:
      motor_commands = self._ClipMotorCommands(motor_commands)

    super(Laikago, self).ApplyAction(motor_commands, motor_control_mode)
    return

  def _ClipMotorCommands(self, motor_commands):
    """Clips motor commands.

    Args:
      motor_commands: np.array. Can be motor angles, torques, hybrid commands,
        or motor pwms (for Minitaur only).

    Returns:
      Clipped motor commands.
    """

    # clamp the motor command by the joint limit, in case weired things happens
    max_angle_change = MAX_MOTOR_ANGLE_CHANGE_PER_STEP
    current_motor_angles = self.GetMotorAngles()
    motor_commands = np.clip(motor_commands,
                             current_motor_angles - max_angle_change,
                             current_motor_angles + max_angle_change)
    return motor_commands

  @classmethod
  def GetConstants(cls):
    del cls
    return laikago_constants

  # The following functions are added for the migration purpose. Will be removed
  # after the migration is complete.

  @property
  def robot_id(self):
    return self.quadruped

  @property
  def base_position(self):
    return self.GetBasePosition()

  @property
  def base_roll_pitch_yaw(self):
    return self.GetTrueBaseRollPitchYaw()

  @property
  def timestamp(self):
    return self.GetTimeSinceReset()
