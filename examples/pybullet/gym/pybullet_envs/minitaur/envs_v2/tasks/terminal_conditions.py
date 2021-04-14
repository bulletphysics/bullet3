"""Contains the terminal conditions for locomotion tasks."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import gin
import numpy as np

from pybullet_envs.minitaur.envs_v2.utilities import minitaur_pose_utils
from pybullet_envs.minitaur.envs_v2.utilities import env_utils_v2 as env_utils
from pybullet_envs.minitaur.envs_v2.utilities import termination_reason as tr


@gin.configurable
def default_terminal_condition_for_minitaur(env):
  """A default terminal condition for Minitaur.

  Minitaur is considered as fallen if the base position is too low or the base
  tilts/rolls too much.

  Args:
    env: An instance of MinitaurGymEnv

  Returns:
    A boolean indicating if Minitaur is fallen.
  """
  orientation = env_utils.get_robot_base_orientation(env.robot)
  rot_mat = env.pybullet_client.getMatrixFromQuaternion(orientation)
  local_up = rot_mat[6:]
  pos = env_utils.get_robot_base_position(env.robot)
  return (np.dot(np.asarray([0, 0, 1]), np.asarray(local_up)) < 0.85 or
          pos[2] < 0.13)


@gin.configurable
def terminal_condition_for_minitaur_extended_env(env):
  """Returns a bool indicating that the extended env is terminated.

  This predicate checks whether 1) the legs are bent inward too much or
  2) the body is tilted too much.

  Args:
    env: An instance of MinitaurGymEnv
  """
  motor_angles = env.robot.motor_angles
  leg_pose = minitaur_pose_utils.motor_angles_to_leg_pose(motor_angles)

  swing_threshold = np.radians(35.0)
  if (leg_pose[0] > swing_threshold or leg_pose[2] > swing_threshold or  # Front
      leg_pose[1] < -swing_threshold or leg_pose[3] < -swing_threshold):  # Rear
    return True
  roll, _, _ = env.robot.base_roll_pitch_yaw
  if abs(roll) > np.radians(30.0):
    return True

  return False


@gin.configurable
def default_terminal_condition_for_laikago(env):
  """A default terminal condition for Laikago.

  Minitaur is considered as fallen if the base position is too low or the base
  tilts/rolls too much.

  Args:
    env: An instance of MinitaurGymEnv

  Returns:
    A boolean indicating if Minitaur is fallen.
  """
  roll, pitch, _ = env.robot.base_roll_pitch_yaw
  pos = env_utils.get_robot_base_position(env.robot)
  return abs(roll) > 0.2 or abs(pitch) > 0.2 or pos[2] < 0.35


@gin.configurable
def default_terminal_condition_for_laikago_v2(
    env,
    max_roll: float = 1.2,
    max_pitch: float = 1.2,
    min_height: float = 0.15,
    enforce_foot_contacts: bool = False):
  """A default terminal condition for Laikago_v2.

  Laikago is considered as fallen if the base position is too low or the base
  tilts/rolls too much.

  Args:
    env: An instance of MinitaurGymEnv
    max_roll:  Max roll before the episode terminates.
    max_pitch:  Max pitch before the episode terminates.
    min_height:  Min height before the episode terminates.
    enforce_foot_contacts: Ensure that contacts are established with the feet.

  Returns:
    A boolean indicating if Minitaur is fallen.
  """
  # Make sure that contacts are only made with the robot's feet.
  unwanted_collision = False
  if enforce_foot_contacts:
    # Get list of foot and knee link ids. Sometimes, the simulation will
    # register a contact as having been made with the knee link, even though it
    # was actually the foot that made the contact. Checking both the foot and
    # knee links for contact accounts for that.
    foot_link_ids = list(
        env.robot.urdf_loader.get_end_effector_id_dict().values())
    knee_link_ids = [foot_link_id - 1 for foot_link_id in foot_link_ids]
    contacts = env.pybullet_client.getContactPoints(bodyA=env.robot.robot_id)
    for contact in contacts:
      # Two different bodies made contact (i.e. not a self-collision).
      if contact[1] != contact[2]:
        foot_contact = (contact[3] in foot_link_ids) or (
            contact[3] in knee_link_ids)
        unwanted_collision = unwanted_collision or not foot_contact

  roll, pitch, _ = env.robot.base_roll_pitch_yaw
  pos = env.robot.base_position
  return (abs(roll) > max_roll or abs(pitch) > max_pitch or
          pos[2] < min_height or unwanted_collision)


@gin.configurable
def default_terminal_condition_for_agility(env,
                                           max_roll: float = 1.8,
                                           max_pitch: float = 1.8,
                                           min_height: float = 0.0,
                                           enforce_foot_contacts: bool = False):
  """A default terminal condition for more agile tasks (i.e. jumping).

  The robot is considered as fallen if the base position is too low, the base
  tilts/rolls too much or parts of the body other than the feet touch the
  ground.

  Args:
    env: An instance of the gym env.
    max_roll:  Max roll before the episode terminates.
    max_pitch:  Max pitch before the episode terminates.
    min_height:  Min height before the episode terminates.
    enforce_foot_contacts: Ensure that contacts are established with the feet.

  Returns:
    A boolean indicating if the episode should be terminated.
  """

  # Make sure that contacts are only made with the robot's feet.
  unwanted_collision = False
  if enforce_foot_contacts:
    knee_link_ids = [2, 5, 8, 11]
    contacts = env.pybullet_client.getContactPoints(bodyA=env.robot.robot_id)
    for contact in contacts:
      if contact[1] != contact[2]:
        foot_contact = contact[3] in knee_link_ids
        unwanted_collision = unwanted_collision or not foot_contact

  roll, pitch, _ = env.robot.base_roll_pitch_yaw
  pos = env.robot.base_position
  return (abs(roll) > max_roll or abs(pitch) > max_pitch or
          pos[2] < min_height or unwanted_collision)


@gin.configurable
def get_terminal_reason(collisions, task):
  termination_reason = None
  # Checking collision termination
  if collisions:
    termination_reason = tr.TerminationReason.AGENT_COLLISION
  if task.is_task_success():
    termination_reason = tr.TerminationReason.GOAL_REACHED
  return termination_reason


@gin.configurable
def maxstep_terminal_condition(env, max_step=500):
  """A terminal condition based on the time step.

  Args:
    env: An instance of MinitaurGymEnv
    max_step: The maximum time step allowed for the environment

  Returns:
    A boolean indicating if the env.step exceeds the given limit
  """
  return env.env_step_counter > max_step


@gin.configurable
class MaxTimeTerminalCondition(object):
  """Terminal condition based on time, independent of step length."""

  def __init__(self, max_time_s: float):
    """Initializes the MaxTimeTerminalCondition.

    Args:
      max_time_s: Time limit in seconds. In sim, this is the simulation time,
        not wall time.
    """
    if max_time_s <= 0:
      raise ValueError("Max time for MaxTimeTerminalCondition cannot be zero "
                       "or less. Input value: %s" % max_time_s)
    self._max_time_s = max_time_s

  def __call__(self, env):
    if self._max_time_s is None:
      return False, None
    is_done = env.get_time_since_reset() >= self._max_time_s
    term_reason = tr.TerminationReason.RUN_TIME_LIMIT if is_done else None
    return is_done, term_reason


@gin.configurable
class MovementDetectorTerminalCondition(object):
  """Terminal condition for not moving past a distance in specified time."""

  def __init__(self,
               max_time_s: float = None,
               min_travel_distance_m: float = 1.0):
    """Initializes the MovementDetectorTerminalCondition.

    Args:
      max_time_s: Time limit in seconds. In sim, this is the simulation time,
        not wall time.
      min_travel_distance_m: Minimum distance in meters to travel in time limit.
    """
    if max_time_s is not None and max_time_s <= 0:
      raise ValueError("Max time for MovementDetectorTerminalCondition cannot "
                       "be zero or less. Input value: %s" % max_time_s)
    if min_travel_distance_m is not None and min_travel_distance_m < 0:
      raise ValueError(
          "Minimum travel distance for MovementDetectorTerminalCondition "
          "cannot be less than zero. Input value: %s" % min_travel_distance_m)
    self._max_time_s = max_time_s
    self._min_travel_distance_m = min_travel_distance_m
    self._not_advancing_limit = None
    self._reference_position = None

  def _update_limit_time(self, env):
    self._not_advancing_limit = env.get_time_since_reset() + self._max_time_s

  def _current_position(self, env):
    return np.asarray(env.robot.base_position[:2])

  def _exceed_time_limit(self, env):
    return self._not_advancing_limit < env.get_time_since_reset()

  def __call__(self, env):
    is_done = False
    term_reason = None

    if self._max_time_s is None:
      return is_done, term_reason

    if self._not_advancing_limit is None or self._reference_position is None:
      self._update_limit_time(env)
      self._reference_position = self._current_position(env)

    distance_shifted = np.linalg.norm(
        self._current_position(env) - self._reference_position)
    if distance_shifted >= self._min_travel_distance_m:
      self._update_limit_time(env)
      self._reference_position = self._current_position(env)

    if self._exceed_time_limit(env):
      is_done = True
      term_reason = tr.TerminationReason.NOT_ADVANCING_LIMIT

    return is_done, term_reason



@gin.configurable
def bad_front_leg_terminal_condition(env, max_angle=0.8):
  """A terminal condition for checking whether front legs are bent backward.

  Args:
    env: An instance of MinitaurGymEnv
    max_angle: The maximum angle allowed for front legs

  Returns:
    A boolean indicating if front legs are bent backward or not
  """
  motor_angles = env.robot.motor_angles
  leg_pose = minitaur_pose_utils.motor_angles_to_leg_pose(motor_angles)
  swing0 = leg_pose[0]
  swing1 = leg_pose[2]
  return swing0 > max_angle or swing1 > max_angle


@gin.configurable
def logical_any_terminal_condition(env, conditions):
  """A logical "Any" operator for terminal conditions.

  Args:
    env: An instance of MinitaurGymEnv
    conditions: a list of terminal conditions

  Returns:
    A boolean indicating if any of terminal conditions is satisfied
  """
  return any([cond(env) for cond in conditions])
