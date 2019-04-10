"""A Raibert style controller for Minitaur."""

import collections
import math
import numpy as np

_NUM_MOTORS = 8
_NUM_LEGS = 4

_UPPER_LEG_LEN = 0.112
_LOWER_SHORT_LEG_LEN = 0.199
_LOWER_LONG_LEG_LEN = 0.2315

DIAGONAL_LEG_PAIR_1 = (0, 3)
DIAGONAL_LEG_PAIR_2 = (1, 2)


class BehaviorParameters(
    collections.namedtuple("BehaviorParameters", [
        "stance_duration", "desired_forward_speed", "turning_speed",
        "standing_height", "desired_incline_angle"
    ])):
  __slots__ = ()

  def __new__(cls,
              stance_duration=0.25,
              desired_forward_speed=0.2,
              turning_speed=0,
              standing_height=0.21,
              desired_incline_angle=0):
    return super(BehaviorParameters, cls).__new__(
        cls, stance_duration, desired_forward_speed, turning_speed,
        standing_height, desired_incline_angle)


def motor_angles_to_leg_pose(motor_angles):
  leg_pose = np.zeros(_NUM_MOTORS)
  for i in range(_NUM_LEGS):
    leg_pose[i] = 0.5 * (-1)**(i // 2) * (
        motor_angles[2 * i + 1] - motor_angles[2 * i])
    leg_pose[_NUM_LEGS + i] = 0.5 * (
        motor_angles[2 * i] + motor_angles[2 * i + 1])
  return leg_pose


def leg_pose_to_motor_angles(leg_pose):
  motor_pose = np.zeros(_NUM_MOTORS)
  for i in range(_NUM_LEGS):
    motor_pose[2 * i] = leg_pose[_NUM_LEGS + i] - (-1)**(i // 2) * leg_pose[i]
    motor_pose[2 * i + 1] = (
        leg_pose[_NUM_LEGS + i] + (-1)**(i // 2) * leg_pose[i])
  return motor_pose


def leg_pose_to_foot_position(leg_pose):
  """The forward kinematics."""
  l1 = _UPPER_LEG_LEN
  l2 = _LOWER_SHORT_LEG_LEN
  l3 = _LOWER_LONG_LEG_LEN

  ext = leg_pose[1]
  alpha = math.asin(l1 * math.sin(ext) / l2)

  sw = leg_pose[0]
  x = l3 * math.sin(alpha + sw) - l1 * math.sin(ext + sw)
  y = l3 * math.cos(alpha + sw) - l1 * math.cos(ext + sw)

  return (x, -y)


def foot_position_to_leg_pose(foot_position):
  """The inverse kinematics."""
  l1 = _UPPER_LEG_LEN
  l2 = _LOWER_SHORT_LEG_LEN
  l3 = _LOWER_LONG_LEG_LEN

  x = foot_position[0]
  y = foot_position[1]
  assert (y < 0)
  hip_toe_sqr = x**2 + y**2
  cos_beta = (l1 * l1 + l3 * l3 - hip_toe_sqr) / (2 * l1 * l3)
  hip_ankle_sqr = l1 * l1 + l2 * l2 - 2 * l1 * l2 * cos_beta
  hip_ankle = math.sqrt(hip_ankle_sqr)
  cos_ext = -(l1 * l1 + hip_ankle_sqr - l2 * l2) / (2 * l1 * hip_ankle)
  ext = math.acos(cos_ext)

  hip_toe = math.sqrt(hip_toe_sqr)
  cos_theta = (hip_toe_sqr + hip_ankle_sqr -
               (l3 - l2)**2) / (2 * hip_ankle * hip_toe)

  assert cos_theta > 0
  theta = math.acos(cos_theta)
  sw = math.asin(x / hip_toe) - theta
  return (-sw, ext)


def foot_horizontal_position_to_leg_swing(foot_horizontal_position,
                                          leg_extension):
  """Computes the target leg swing.

  Sometimes it is more convenient to plan in the hybrid space.
  """

  l1 = _UPPER_LEG_LEN
  l2 = _LOWER_SHORT_LEG_LEN
  l3 = _LOWER_LONG_LEG_LEN
  ext = leg_extension
  alpha = math.asin(l1 / l2 * math.sin(ext))

  toe_hip_orth = l3 * math.sin(alpha) - l1 * math.sin(ext)
  toe_hip_proj = l3 * math.cos(alpha) - l1 * math.cos(ext)

  theta = math.atan(toe_hip_orth / toe_hip_proj)

  # We may allow theta < 0 for backward foot location.
  # assert theta > 0

  toe_hip_len = math.sqrt(toe_hip_orth**2 + toe_hip_proj**2)

  # Cap the foot horizontal (projected) position so the target leg pose is
  # always feasible.
  foot_position = max(
      min(toe_hip_len * 0.8, foot_horizontal_position), -toe_hip_len * 0.5)

  sw_and_theta = math.asin(foot_position / toe_hip_len)

  sw = sw_and_theta - theta

  # TODO(tingnan): Fix the sign bug.
  return -sw


def extension_to_ankle_dist(ext):
  l1 = _UPPER_LEG_LEN
  l2 = _LOWER_SHORT_LEG_LEN
  l3 = _LOWER_LONG_LEG_LEN
  alpha = math.asin(l1 / l2 * math.sin(ext))
  return l2 * math.cos(alpha) - l1 * math.cos(ext)


def ankle_dist_to_extension(dist):
  l1 = _UPPER_LEG_LEN
  l2 = _LOWER_SHORT_LEG_LEN
  l3 = _LOWER_LONG_LEG_LEN
  cos_ext = -(l1**2 + dist**2 - l2**2) / (2 * l1 * dist)
  return math.acos(cos_ext)


def generate_swing_trajectory(phase, init_pose, end_pose):
  # Try phase compression
  normalized_phase = math.sqrt(min(phase * 1.5, 1))

  # For swing, we use a linear interpolation:
  sw = (end_pose[0] - init_pose[0]) * normalized_phase + init_pose[0]

  # For extension, we can fit a second order polynomial:
  min_ext = (init_pose[1] + end_pose[1]) / 2 - 0.8
  min_ext = max(min_ext, 0.5)
  phi = 0.7

  min_delta = extension_to_ankle_dist(min_ext)
  init_delta = extension_to_ankle_dist(init_pose[1])
  end_delta = extension_to_ankle_dist(end_pose[1])

  # The polynomial is: a * phi^2 + b * phi + c
  delta_1 = min_delta - init_delta
  delta_2 = end_delta - init_delta
  delta_p = phi * phi - phi

  a = (delta_1 - phi * delta_2) / delta_p

  b = (phi * phi * delta_2 - delta_1) / delta_p

  delta = (
      a * normalized_phase * normalized_phase + b * normalized_phase +
      init_delta)

  l1 = _UPPER_LEG_LEN
  l2 = _LOWER_SHORT_LEG_LEN

  delta = min(max(delta, l2 - l1 + 0.01), l2 + l1 - 0.01)

  ext = ankle_dist_to_extension(delta)

  return (sw, ext)


def generate_stance_trajectory(phase, init_pose, end_pose):
  normalized_phase = math.sqrt(phase)
  sw = (end_pose[0] - init_pose[0]) * normalized_phase + init_pose[0]
  ext = end_pose[1]
  return (sw, ext)


class RaibertSwingLegController(object):

  def __init__(self,
               speed_gain=0.05,
               leg_extension_clearance=0.3,
               leg_trajectory_generator=generate_swing_trajectory):
    self._speed_gain = speed_gain
    self._leg_extension_clearance = leg_extension_clearance
    self._leg_trajectory_generator = leg_trajectory_generator

  def get_action(self, raibiert_controller):
    current_speed = raibiert_controller.estimate_base_velocity()
    phase = raibiert_controller.get_phase()

    leg_pose_set = []
    for i in raibiert_controller.swing_set:
      target_foot_horizontal_position = (
          raibiert_controller.behavior_parameters.stance_duration / 2 *
          current_speed + self._speed_gain *
          (current_speed -
           raibiert_controller.behavior_parameters.desired_forward_speed))

      # Use the swing phase [0, 1] to track the foot. The idea is
      # straightforward:
      # 1) Calculate the target swing and leg extension based on target foot position.
      # 2) Generate a smooth Bezier curve between current leg pose and target pose
      # 3) Find the next leg pose on the curve based on how much time left.

      # 1) Convert the target foot
      target_leg_extension = (
          raibiert_controller.nominal_leg_extension -
          self._leg_extension_clearance)
      target_leg_swing = foot_horizontal_position_to_leg_swing(
          target_foot_horizontal_position, leg_extension=target_leg_extension)

      target_leg_pose = (target_leg_swing, target_leg_extension)

      # 2) Generates the curve from the current leg pose to the target leg pose.
      # and Find the next leg pose on the curve based on current swing phase.

      desired_leg_pose = self._leg_trajectory_generator(
          phase, raibiert_controller.swing_start_leg_pose, target_leg_pose)

      leg_pose_set.append(desired_leg_pose)

    # 3) adjust the pose with a feedback term to maintain leg height

    return leg_pose_set


class RaibertStanceLegController(object):

  def __init__(self,
               speed_gain=0.1,
               leg_trajectory_generator=generate_stance_trajectory):
    self._speed_gain = speed_gain
    self._leg_trajectory_generator = leg_trajectory_generator

  def get_action(self, raibiert_controller):
    phase = raibiert_controller.get_phase()
    current_speed = raibiert_controller.estimate_base_velocity()
    leg_pose_set = []
    for i in raibiert_controller.stance_set:
      desired_forward_speed = (
          raibiert_controller.behavior_parameters.desired_forward_speed)
      target_foot_position = -(
          raibiert_controller.behavior_parameters.stance_duration / 2 *
          current_speed - self._speed_gain *
          (current_speed - desired_forward_speed))

      target_leg_pose = (foot_horizontal_position_to_leg_swing(
          target_foot_position,
          leg_extension=raibiert_controller.nominal_leg_extension),
                         raibiert_controller.nominal_leg_extension)

      desired_leg_pose = self._leg_trajectory_generator(
          phase, raibiert_controller.stance_start_leg_pose, target_leg_pose)

      leg_pose_set.append(desired_leg_pose)

    return leg_pose_set


class MinitaurRaibertTrottingController(object):
  """A Raibert style controller for trotting gait."""

  def __init__(self,
               robot,
               behavior_parameters=BehaviorParameters(),
               swing_leg_controller=RaibertSwingLegController(),
               stance_leg_controller=RaibertStanceLegController(),
               pose_feedback_controller=None):
    self._time = 0
    self._robot = robot
    self._behavior_parameters = behavior_parameters

    nominal_leg_pose = foot_position_to_leg_pose(
        (0, -self._behavior_parameters.standing_height))
    self._nominal_leg_extension = nominal_leg_pose[1]

    self._swing_leg_controller = swing_leg_controller
    self._stance_leg_controller = stance_leg_controller
    self._pose_feeback_controller = pose_feedback_controller

    # The leg order is FL, RL, FR, RR -> [0, 1, 2, 3]
    self._swing_set = DIAGONAL_LEG_PAIR_1
    self._stance_set = DIAGONAL_LEG_PAIR_2
    self._swing_start_leg_pose = self.get_swing_leg_pose()
    self._stance_start_leg_pose = self.get_stance_leg_pose()

  @property
  def behavior_parameters(self):
    return self._behavior_parameters

  @behavior_parameters.setter
  def behavior_parameters(self, behavior_parameters):
    self._behavior_parameters = behavior_parameters

  @property
  def nominal_leg_extension(self):
    return self._nominal_leg_extension

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
    current_leg_pose = motor_angles_to_leg_pose(self._robot.GetMotorAngles())

    # extract the swing leg pose from the current_leg_pose
    leg_pose = []
    for index in leg_indices:
      leg_pose.append(
          [current_leg_pose[index], current_leg_pose[index + _NUM_LEGS]])

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
    return math.fmod(self._time, self._behavior_parameters.stance_duration
                    ) / self._behavior_parameters.stance_duration

  def update_swing_stance_set(self):
    """Switch the set of swing/stance legs based on timing."""
    swing_stance_phase = math.fmod(
        self._time, 2 * self._behavior_parameters.stance_duration)
    if swing_stance_phase < self._behavior_parameters.stance_duration:
      return (DIAGONAL_LEG_PAIR_1, DIAGONAL_LEG_PAIR_2)
    return (DIAGONAL_LEG_PAIR_2, DIAGONAL_LEG_PAIR_1)

  def update(self, t):
    self._time = t
    new_swing_set, new_stance_set = self.update_swing_stance_set()

    # If there is a stance/swing switch.
    if new_swing_set[0] is not self._swing_set[0]:
      self._swing_set = new_swing_set
      self._stance_set = new_stance_set

      # Also records the starting pose.
      self._swing_start_leg_pose = self.get_swing_leg_pose()
      self._stance_start_leg_pose = self.get_stance_leg_pose()

  def estimate_base_velocity(self):
    # TODO(tingnan): consider using a sensor fusion.
    stance_leg_pose = self.get_stance_leg_pose()

    delta_sw = stance_leg_pose[0] - self._stance_start_leg_pose[0]

    x, y = leg_pose_to_foot_position(stance_leg_pose)
    toe_hip_len = math.sqrt(x**2 + y**2)
    horizontal_dist = toe_hip_len * delta_sw
    phase = self.get_phase()
    speed = 0 if phase < 0.1 else horizontal_dist / (
        self._behavior_parameters.stance_duration * phase)
    return speed

  def get_swing_leg_action(self):
    return self._swing_leg_controller.get_action(self)

  def get_stance_leg_action(self):
    return self._stance_leg_controller.get_action(self)

  def get_action(self):
    leg_pose = [0] * _NUM_MOTORS
    swing_leg_pose = self.get_swing_leg_action()
    j = 0
    for i in self._swing_set:
      leg_pose[i] = swing_leg_pose[j][0]
      leg_pose[i + _NUM_LEGS] = swing_leg_pose[j][1]
      j += 1

    stance_leg_pose = self.get_stance_leg_action()
    j = 0
    for i in self._stance_set:
      leg_pose[i] = stance_leg_pose[j][0]
      leg_pose[i + _NUM_LEGS] = stance_leg_pose[j][1]
      j += 1

    return leg_pose_to_motor_angles(leg_pose)
