"""A Raibert style controller for Minitaur."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import math
import attr
import gin
import numpy as np

from pybullet_envs.minitaur.agents.baseline_controller import minitaur_raibert_controller_utils
from pybullet_envs.minitaur.envs.utilities import minitaur_pose_utils

LEFT_FRONT_LEG_ID = 0
LEFT_HIND_LEG_ID = 1
RIGHT_FRONT_LEG_ID = 2
RIGHT_HIND_LEG_ID = 3

DIAGONAL_LEG_PAIR_1 = (LEFT_FRONT_LEG_ID, RIGHT_HIND_LEG_ID)
DIAGONAL_LEG_PAIR_2 = (LEFT_HIND_LEG_ID, RIGHT_FRONT_LEG_ID)

LEFT_LEG_IDS = (LEFT_FRONT_LEG_ID, LEFT_HIND_LEG_ID)
RIGHT_LEG_IDS = (RIGHT_FRONT_LEG_ID, RIGHT_HIND_LEG_ID)

# The max horizontal foot offset (in meters) for turning.
_FOOT_HORIZONTAL_OFFSET_FOR_TURNING = 0.1

_STANCE_TG_PHASE_COMPRESSION = 1.5
_STANCE_TG_DELTA_EXTENSION = 0.2
_STANCE_HORIZONTAL_SCALING_FACTOR = 1.2

_DEFAULT_SWING_SPEED_GAIN = 0.015

_DEFAULT_SWING_FOOT_CLEARANCE = 0.005

_PITCH_SWING_FEEDBACK_SCALING_FACTOR = 1.2


# A POD container to describe the controller's high level behavior.
@attr.s
class BehaviorParameters(object):
  """Highlevel parameters for Raibert style controller."""
  stance_duration = attr.ib(type=float, default=0.25)
  desired_forward_speed = attr.ib(type=float, default=0.)
  desired_turning_speed = attr.ib(type=float, default=0.)
  standing_height = attr.ib(type=float, default=0.2)
  desired_incline_angle = attr.ib(type=float, default=0.)


def generate_default_swing_trajectory(phase, init_pose, end_pose):
  """A swing trajectory generator.

  Args:
    phase: Float. Between [0, 1].
    init_pose: A tuple. The leg pose (swing, extension) at phase == 0 the
      beginning of swing.
    end_pose: A tuple. The leg pose at phase == 1 the end of swing.

  Returns:
    The desired leg pose for the current phase.
  """
  # Try phase compression
  normalized_phase = math.sqrt(min(phase * 1.5, 1))

  # For swing, we use a linear interpolation:
  swing = (end_pose[0] - init_pose[0]) * normalized_phase + init_pose[0]

  # For extension, we can fit a second order polynomial:
  min_ext = (init_pose[1] + end_pose[1]) / 2 - 0.8
  min_ext = max(min_ext, 0.5)

  # The phase value at which the extension reaches the minimal value min_ext.
  # phi is small, the swing leg will try to lift higher in the first half of
  # swing.
  phi = 0.1

  # We convert the extension back into the cartesion space. In this way we can
  # guarantee a lift-up trajectory. The ankle to hip distance is easier to
  # compute than a full forward-kinematics.
  min_ankle_dist = minitaur_raibert_controller_utils.extension_to_ankle_dist(
      min_ext)
  init_ankle_dist = minitaur_raibert_controller_utils.extension_to_ankle_dist(
      init_pose[1])
  end_ankle_dist = minitaur_raibert_controller_utils.extension_to_ankle_dist(
      end_pose[1])

  # The polynomial is: a * phi^2 + b * phi + c
  delta_1 = min_ankle_dist - init_ankle_dist
  delta_2 = end_ankle_dist - init_ankle_dist
  delta_p = phi * phi - phi

  a = (delta_1 - phi * delta_2) / delta_p

  b = (phi * phi * delta_2 - delta_1) / delta_p

  ankle_dist = (
      a * normalized_phase * normalized_phase + b * normalized_phase +
      init_ankle_dist)

  l1 = minitaur_raibert_controller_utils.UPPER_LEG_LEN
  l2 = minitaur_raibert_controller_utils.LOWER_SHORT_LEG_LEN

  ankle_dist = min(max(ankle_dist, l2 - l1 + 0.01), l2 + l1 - 0.01)

  extension = minitaur_raibert_controller_utils.ankle_dist_to_extension(
      ankle_dist)

  return (swing, extension)


@gin.configurable
def generate_default_stance_trajectory(phase,
                                       init_pose,
                                       end_pose,
                                       use_constant_extension=False):
  """A stance strajectory generator.

  Args:
    phase: Float. Between [0, 1].
    init_pose: A tuple. The leg pose (swing, extension) at phase == 0, i.e. the
      beginning of stance.
    end_pose: A tuple. The leg pose at phase == 1, i.e. the end of stance.
    use_constant_extension: Boolean. Whether or not to fix the extension during
      stance.

  Returns:
    The desired leg pose for the current phase.
  """
  normalized_phase = min(_STANCE_TG_PHASE_COMPRESSION * math.sqrt(phase), 1)
  swing = (end_pose[0] - init_pose[0]) * normalized_phase + init_pose[0]

  # The extension evolves nonlinearly according to the parabola equation.
  if use_constant_extension:
    extension = end_pose[1]
  else:
    extension = end_pose[1] - 4 * _STANCE_TG_DELTA_EXTENSION * (
        normalized_phase**2 - normalized_phase)
  return (swing, extension)


def get_stance_foot_offset_for_turning(leg_id, steering_signal):
  """Modify the stance foot position to achieve turning.

  The strategy works for trotting gaits.

  Args:
    leg_id: Integer. The leg index.
    steering_signal: Float. The desired turning signal in [-1, 1]. Because we
      don't have an accurate mapping from angular velocity to the foot offset,
      the steering signal should be treated as a reference and only its relative
      magnitude matters.

  Returns:
    The stance foot's horizontal offset.

  """
  clipped_steering_signal = np.clip(steering_signal, -1, 1)

  if leg_id in LEFT_LEG_IDS:
    return _FOOT_HORIZONTAL_OFFSET_FOR_TURNING * clipped_steering_signal
  else:
    return -(_FOOT_HORIZONTAL_OFFSET_FOR_TURNING * clipped_steering_signal)


def get_leg_swing_offset_for_pitching(body_pitch, desired_incline_angle):
  """Get the leg swing zero point when the body is tilted.

  For example, when climbing up or down stairs/slopes, the robot body will tilt
  up or down. By compensating the body pitch, the leg's trajectory will be
  centered around the vertical direction (not perpendicular to the surface).
  This helps the robot to generate thrust when going upwards, and braking when
  going downwards.

  Args:
    body_pitch: Float. Current body pitch angle.
    desired_incline_angle: Float. The desired body pitch angle.

  Returns:
    The stance and swing leg swing offset.

  """
  kp = 0.2
  return -((1 - kp) * body_pitch + kp * desired_incline_angle)


@gin.configurable
class RaibertSwingLegController(object):
  """The swing leg controller."""

  def __init__(self,
               speed_gain=_DEFAULT_SWING_SPEED_GAIN,
               foot_clearance=_DEFAULT_SWING_FOOT_CLEARANCE,
               leg_trajectory_generator=generate_default_swing_trajectory):
    """Initializes the controller.

    Args:
      speed_gain: Float. The speed feedback gain to modulate the target foot
        position.
      foot_clearance: Float. The foot clearance (at the end of the swing phase)
        in meters.
      leg_trajectory_generator: A trajectory generator function.
    """
    self._speed_gain = speed_gain
    self._foot_clearance = foot_clearance
    self._leg_trajectory_generator = leg_trajectory_generator

  def get_action(self, raibert_controller):
    """Get the swing legs' desired pose."""
    current_speed = raibert_controller.estimate_base_velocity()
    phase = raibert_controller.get_phase()
    rpy = raibert_controller.robot.base_roll_pitch_yaw

    leg_pose_set = {}
    for i in raibert_controller.swing_set:
      # Target foot horizontal position is calculated according to Raibert's
      # original formula in "Legged robots that balance".
      target_foot_horizontal_position = (
          raibert_controller.behavior_parameters.stance_duration / 2 *
          current_speed + self._speed_gain *
          (current_speed -
           raibert_controller.behavior_parameters.desired_forward_speed))

      # 1) Convert the target foot position to leg pose space.
      # Lift the foot a little bit.
      target_foot_vertical_position = -(
          raibert_controller.behavior_parameters.standing_height -
          self._foot_clearance)
      target_foot_position = (target_foot_horizontal_position,
                              target_foot_vertical_position)
      target_leg_pose = minitaur_raibert_controller_utils.foot_position_to_leg_pose(
          target_foot_position)

      # 2) Generates the curve from the swing start leg pose to the target leg
      # pose and find the next leg pose on the curve based on current swing
      # phase.

      desired_leg_pose = self._leg_trajectory_generator(
          phase, raibert_controller.swing_start_leg_pose, target_leg_pose)

      swing_offset = get_leg_swing_offset_for_pitching(
          rpy[1], raibert_controller.behavior_parameters.desired_incline_angle)

      leg_pose_set[i] = (desired_leg_pose[0] + swing_offset,
                         desired_leg_pose[1])

    return leg_pose_set


@gin.configurable
class RaibertStanceLegController(object):
  """The controller that modulates the behavior of the stance legs."""

  def __init__(self,
               speed_gain=0.1,
               leg_trajectory_generator=generate_default_stance_trajectory):
    """Initializes the controller.

    Args:
      speed_gain: Float. The speed feedback gain to modulate the target stance
        foot position.
      leg_trajectory_generator: A trajectory generator function. Generates the
        desired leg pose during the stance phase.
    """
    self._speed_gain = speed_gain
    self._leg_trajectory_generator = leg_trajectory_generator

  def get_action(self, raibert_controller):
    """Get the desired leg pose for the stance legs."""

    phase = raibert_controller.get_phase()
    current_speed = raibert_controller.estimate_base_velocity()
    rpy = raibert_controller.robot.base_roll_pitch_yaw

    leg_pose_set = {}
    for i in raibert_controller.stance_set:
      desired_forward_speed = (
          raibert_controller.behavior_parameters.desired_forward_speed)

      target_foot_horizontal_position = -_STANCE_HORIZONTAL_SCALING_FACTOR * (
          raibert_controller.behavior_parameters.stance_duration / 2 *
          current_speed - self._speed_gain *
          (current_speed - desired_forward_speed))

      target_foot_horizontal_position += get_stance_foot_offset_for_turning(
          i, raibert_controller.behavior_parameters.desired_turning_speed)

      target_foot_position = (
          target_foot_horizontal_position,
          -raibert_controller.behavior_parameters.standing_height)
      target_leg_pose = minitaur_raibert_controller_utils.foot_position_to_leg_pose(
          target_foot_position)

      desired_leg_pose = (
          self._leg_trajectory_generator(
              phase, raibert_controller.stance_start_leg_pose, target_leg_pose))

      swing_offset = _PITCH_SWING_FEEDBACK_SCALING_FACTOR * get_leg_swing_offset_for_pitching(
          rpy[1], raibert_controller.behavior_parameters.desired_incline_angle)

      leg_pose_set[i] = (desired_leg_pose[0] + swing_offset,
                         desired_leg_pose[1])

    return leg_pose_set


@gin.configurable
class MinitaurRaibertController(object):
  """A Raibert style controller for trotting gait."""

  def __init__(self,
               robot,
               behavior_parameters=BehaviorParameters(),
               swing_leg_controller=RaibertSwingLegController(),
               stance_leg_controller=RaibertStanceLegController(),
               pose_feedback_controller=None):
    self._time = 0
    self._robot = robot
    self.behavior_parameters = behavior_parameters

    self._last_recorded_speed = 0

    self._swing_leg_controller = swing_leg_controller
    self._stance_leg_controller = stance_leg_controller
    self._pose_feeback_controller = pose_feedback_controller

    # The leg order is FL, RL, FR, RR -> [0, 1, 2, 3]
    self._swing_set = DIAGONAL_LEG_PAIR_1
    self._stance_set = DIAGONAL_LEG_PAIR_2

    # Compute the initial leg pose.
    self._swing_start_leg_pose = self.get_swing_leg_pose()
    self._stance_start_leg_pose = self.get_stance_leg_pose()

  @property
  def robot(self):
    return self._robot

  @property
  def swing_set(self):
    return self._swing_set

  @property
  def stance_set(self):
    return self._stance_set

  @property
  def swing_start_leg_pose(self):
    return self._swing_start_leg_pose

  @property
  def stance_start_leg_pose(self):
    return self._stance_start_leg_pose

  def _get_average_leg_pose(self, leg_indices):
    """Get the average leg pose."""
    current_leg_pose = minitaur_pose_utils.motor_angles_to_leg_pose(
        self._robot.motor_angles)

    # extract the swing leg pose from the current_leg_pose
    leg_pose = []
    for index in leg_indices:
      leg_pose.append([
          current_leg_pose[index],
          current_leg_pose[index + minitaur_pose_utils.NUM_LEGS]
      ])

    leg_pose = np.array(leg_pose)
    return np.mean(leg_pose, axis=0)

  def get_swing_leg_pose(self):
    """Get the current swing legs' average pose."""
    return self._get_average_leg_pose(self._swing_set)

  def get_stance_leg_pose(self):
    """Get the current stance legs' average pose."""
    return self._get_average_leg_pose(self._stance_set)

  def get_phase(self):
    """Compute the current stance/swing phase."""
    return math.fmod(self._time, self.behavior_parameters.stance_duration
                    ) / self.behavior_parameters.stance_duration

  def _get_new_swing_stance_set(self):
    """Switch the set of swing/stance legs based on timing."""
    swing_stance_phase = math.fmod(self._time,
                                   2 * self.behavior_parameters.stance_duration)
    if swing_stance_phase < self.behavior_parameters.stance_duration:
      return (DIAGONAL_LEG_PAIR_1, DIAGONAL_LEG_PAIR_2)
    return (DIAGONAL_LEG_PAIR_2, DIAGONAL_LEG_PAIR_1)

  def update(self, t):
    """Update the internal status of the controller.

    Args:
      t: Float. The current time after reset in seconds.
    """
    self._time = t
    new_swing_set, new_stance_set = self._get_new_swing_stance_set()

    # If there is a stance/swing switch.
    if new_swing_set[0] is not self._swing_set[0]:
      self._swing_set = new_swing_set
      self._stance_set = new_stance_set

      # Also records the starting pose.
      self._swing_start_leg_pose = self.get_swing_leg_pose()
      self._stance_start_leg_pose = self.get_stance_leg_pose()

  def estimate_base_velocity(self):
    """Estimate the current CoM speed."""
    # It is best to use a sensor fusion approach.
    stance_leg_pose = self.get_stance_leg_pose()

    delta_sw = stance_leg_pose[0] - self._stance_start_leg_pose[0]

    x, y = minitaur_raibert_controller_utils.leg_pose_to_foot_position(
        stance_leg_pose)
    toe_hip_len = math.sqrt(x**2 + y**2)
    horizontal_dist = toe_hip_len * delta_sw
    phase = self.get_phase()
    speed = self._last_recorded_speed
    if phase > 0.1:
      speed = horizontal_dist / (
          self.behavior_parameters.stance_duration * phase)
      self._last_recorded_speed = speed
    return speed

  def get_swing_leg_action(self):
    return self._swing_leg_controller.get_action(self)

  def get_stance_leg_action(self):
    return self._stance_leg_controller.get_action(self)

  def get_action(self):
    """Gets the desired motor angles."""
    leg_pose = [0] * minitaur_pose_utils.NUM_MOTORS
    swing_leg_pose = self.get_swing_leg_action()
    for i in self._swing_set:
      leg_pose[i] = swing_leg_pose[i][0]
      leg_pose[i + minitaur_pose_utils.NUM_LEGS] = swing_leg_pose[i][1]

    stance_leg_pose = self.get_stance_leg_action()
    for i in self._stance_set:
      leg_pose[i] = stance_leg_pose[i][0]
      leg_pose[i + minitaur_pose_utils.NUM_LEGS] = stance_leg_pose[i][1]

    return minitaur_pose_utils.leg_pose_to_motor_angles(leg_pose)
