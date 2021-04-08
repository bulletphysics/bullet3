"""Pybullet simulation of a vision60 robot."""
import math
import os

import gin
import numpy as np

from pybullet_envs.minitaur.robots import laikago_motor
from pybullet_envs.minitaur.robots import minitaur
from pybullet_envs.minitaur.robots import robot_config

NUM_MOTORS = 12
NUM_LEGS = 4
MOTOR_NAMES = [
    "torso_to_abduct_fl_j",  # Left front abduction (hip0).
    "abduct_fl_to_thigh_fl_j",  # Left front hip (upper0).
    "thigh_fl_to_knee_fl_j",  # Left front knee (lower0).
    "torso_to_abduct_hl_j",  # Left rear abduction (hip1).
    "abduct_hl_to_thigh_hl_j",  # Left rear hip (upper1).
    "thigh_hl_to_knee_hl_j",  # Left rear knee (lower1).
    "torso_to_abduct_fr_j",  # Right front abduction (hip2).
    "abduct_fr_to_thigh_fr_j",  # Right front hip (upper2).
    "thigh_fr_to_knee_fr_j",  # Right front knee (lower2).
    "torso_to_abduct_hr_j",  # Right rear abduction (hip3).
    "abduct_hr_to_thigh_hr_j",  # Right rear hip (upper3).
    "thigh_hr_to_knee_hr_j",  # Right rear knee (lower3).
]
_DEFAULT_TORQUE_LIMITS = [12, 18, 12] * 4
INIT_RACK_POSITION = [0, 0, 1.4]
INIT_POSITION = [0, 0, 0.4]
JOINT_DIRECTIONS = np.array([1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
HIP_JOINT_OFFSET = 0.0
UPPER_LEG_JOINT_OFFSET = 0.0
KNEE_JOINT_OFFSET = 0.0
DOFS_PER_LEG = 3
JOINT_OFFSETS = np.array(
    [HIP_JOINT_OFFSET, UPPER_LEG_JOINT_OFFSET, KNEE_JOINT_OFFSET] * 4)
PI = math.pi
DEFAULT_ABDUCTION_ANGLE = 0.0
DEFAULT_HIP_ANGLE = -1.1
DEFAULT_KNEE_ANGLE = 2.3
# Bases on the readings from 's default pose.
INIT_MOTOR_ANGLES = [
    DEFAULT_ABDUCTION_ANGLE, DEFAULT_HIP_ANGLE, DEFAULT_KNEE_ANGLE
] * NUM_LEGS
DEFAULT_LOCAL_TOE_POSITIONS = [[0.17, -0.11, -0.16], [0.17, 0.11, -0.16],
                               [-0.20, -0.11, -0.16], [-0.20, 0.11, -0.16]]


@gin.configurable
class MiniCheetah(minitaur.Minitaur):
  """A simulation for the mini cheetah robot."""

  def __init__(self, **kwargs):
    if "motor_kp" not in kwargs:
      kwargs["motor_kp"] = 100.0
    if "motor_kd" not in kwargs:
      kwargs["motor_kd"] = 2.0
    if "motor_torque_limits" not in kwargs:
      kwargs["motor_torque_limits"] = _DEFAULT_TORQUE_LIMITS

    # The follwing parameters are fixed for the vision60 robot.
    kwargs["num_motors"] = NUM_MOTORS
    kwargs["dofs_per_leg"] = DOFS_PER_LEG
    kwargs["motor_direction"] = JOINT_DIRECTIONS
    kwargs["motor_offset"] = JOINT_OFFSETS
    kwargs["motor_overheat_protection"] = False
    kwargs["motor_model_class"] = laikago_motor.LaikagoMotorModel
    super(MiniCheetah, self).__init__(**kwargs)

  def _LoadRobotURDF(self):
    mini_cheetah_urdf_path = "mini_cheetah/mini_cheetah.urdf"
    if self._self_collision_enabled:
      self.quadruped = self._pybullet_client.loadURDF(
          mini_cheetah_urdf_path,
          self._GetDefaultInitPosition(),
          self._GetDefaultInitOrientation(),
          flags=self._pybullet_client.URDF_USE_SELF_COLLISION)
    else:
      self.quadruped = self._pybullet_client.loadURDF(
          mini_cheetah_urdf_path, self._GetDefaultInitPosition(),
          self._GetDefaultInitOrientation())

  def _SettleDownForReset(self, default_motor_angles, reset_time):
    self.ReceiveObservation()
    for _ in range(500):
      self.ApplyAction(
          INIT_MOTOR_ANGLES,
          motor_control_mode=robot_config.MotorControlMode.POSITION)
      self._pybullet_client.stepSimulation()
      self.ReceiveObservation()
    if default_motor_angles is not None:
      num_steps_to_reset = int(reset_time / self.time_step)
      for _ in range(num_steps_to_reset):
        self.ApplyAction(
            default_motor_angles,
            motor_control_mode=robot_config.MotorControlMode.POSITION)
        self._pybullet_client.stepSimulation()
        self.ReceiveObservation()

  def GetURDFFile(self):
    return os.path.join(self._urdf_root, "mini_cheetah/mini_cheetah.urdf")

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
      angle = INIT_MOTOR_ANGLES[i]
      self._pybullet_client.resetJointState(
          self.quadruped, self._joint_name_to_id[name], angle, targetVelocity=0)

  def _BuildUrdfIds(self):
    pass

  def _GetMotorNames(self):
    return MOTOR_NAMES

  def _GetDefaultInitPosition(self):
    if self._on_rack:
      return INIT_RACK_POSITION
    else:
      return INIT_POSITION

  def _GetDefaultInitOrientation(self):
    init_orientation = [0, 0, 0, 1.0]
    return init_orientation
