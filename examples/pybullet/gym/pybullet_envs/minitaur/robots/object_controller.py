# Lint as: python3
"""Module for controllers of autonomous objects."""

import abc
import bisect
import enum
from typing import Any, Dict, Optional, Sequence, Text, Tuple, Union

from absl import logging
import dataclasses
import gin
import numpy as np


# A constant to be passed into act as parameter t for initial value.
INIT_TIME = -1.0

# Distance that is deemed close enough in ChaseController.
_EPS_DISTANCE = 1e-4

ControllerOutput = Tuple[np.ndarray, np.ndarray, Dict[Text, Any]]

ANIMATION_FRAME_NUMBER_KEY = "animation_frame_number"


class ControllerBase(metaclass=abc.ABCMeta):
  """Base class of object controllers.

  Controller is similar to a policy in that its output controls autonomous
  object just as policy output controls agent. To reflect this similarity,
  get_action(), the function that "commands" to autonomous object, is named
  similar to the counterpart in policy.
  """

  @abc.abstractmethod
  def get_action(self,
                 time_sec: float,
                 observations: Dict[Text, Any]) -> ControllerOutput:
    """Returns position, orientation and pose based on time and observations.

    Args:
      time_sec: Time since simulation reset in seconds. If time < 0, returns
        initial values and ignores observations.
      observations: A dict of all observations.

    Returns:
      Position, orientation and an extra info dict for robot joints, human
        skeletal pose, etc.
    """


@gin.configurable
class StationaryController(ControllerBase):
  """Controller that keeps constant position and orientation."""

  def __init__(self,
               position: Sequence[float] = None,
               orientation: Sequence[float] = None):
    self._position = np.array(position if position is not None else (0, 0, 0))
    self._orientation = np.array(
        orientation if orientation is not None else (0, 0, 0, 1))

  def get_action(self,
                 t: float,
                 observations: Dict[Text, Any]) -> ControllerOutput:
    """Returns constant position orientation."""
    del t, observations
    return self._position, self._orientation, {}


@gin.configurable
class CircularMotionController(ControllerBase):
  """Controller for circular motion.

  The motion trajectory goes around a center in a circle in xy-plane.
  """

  def __init__(self,
               center: Sequence[float],
               radius: float,
               angular_velocity: float = np.pi,
               face_travel_direction: bool = False):
    """Constructor.

    Args:
      center: Center of circular motion, [x, y, z] in meters.
      radius: Radius of the circle in meters.
      angular_velocity: Angular velocity of motion, unit rad/s, e.g. pi means
        completing a circle in 2 sec.
      face_travel_direction: If True, object will face direction of motion.
    """

    self._center = np.array(center)
    self._radius = radius
    self._angular_velocity = angular_velocity
    self._face_travel_direction = face_travel_direction

  def get_action(self,
                 t: float,
                 observations: Dict[Text, Any]) -> ControllerOutput:
    """Returns position on the circle based on time and constant orientation."""
    del observations

    t = max(0.0, t)
    position = np.array(
        [np.cos(self._angular_velocity * t),
         np.sin(self._angular_velocity * t),
         0]) * self._radius + self._center
    if self._face_travel_direction:
      yaw = self._angular_velocity * t + (
          np.sign(self._angular_velocity) * np.pi / 2)
      orientation = np.array((0, 0, np.sin(yaw / 2), np.cos(yaw / 2)))
    else:
      orientation = np.array((0, 0, 0, 1))
    return position, orientation, {}


class PatrolRepeatMode(enum.Enum):
  """Enums that defines trajectory repeat mode for patrol type controller."""

  # Trajectory does not repeat.  For 3 points a, b, c, the trajectory moves
  # along a -> b -> c and then stops at c forever.
  NO_REPEAT = 0

  # Trajectory repeats as a loop. For 3 points a, b, c, the trajectory moves
  # along a -> b -> c -> a -> b ...
  LOOP = 1

  # Trajectory repeats back tracking previous point first. For 3 points a, b, c,
  # the trajectory moves along a -> b -> c -> b -> a -> b -> c ...
  BACK_TRACK = 2

  # Trajectory repeats by resetting to the initial position after reaching end.
  # For 3 points a, b, c, the trajectory moves a -> b -> c then immediately
  # jumps back to a before continue moving along a -> b -> c again.
  RESET = 3


@dataclasses.dataclass
class PatrolSegmentData:
  """A data class that describes a patrol segment."""

  # Time in a single cycle to start this segment, range [0, cycle_time).
  start_time: float

  # Segment start position.
  start_position: np.ndarray

  # Segment velocity vector.
  velocity: np.ndarray

  # Orientation quaternion of this segment.
  orientation: np.ndarray


@gin.configurable
class WayPointPatrolController(ControllerBase):
  """Controller for patrolling along define waypoints."""

  def __init__(self,
               points: Sequence[Sequence[float]],
               yaw_angle: float = 0,
               face_travel_direction: bool = True,
               repeat_mode: Union[
                   PatrolRepeatMode, Text] = PatrolRepeatMode.NO_REPEAT,
               speed_mps: Optional[float] = 1.0,
               time_points: Optional[Sequence[float]] = None):
    """Constructor.

    Args:
      points: List of waypoints, shape Nx3 or Nx2, N is number of points.
      yaw_angle: Yaw angle of the object in radians.
      face_travel_direction: If True, yaw angle 'zero' will be redefined to be
        object's travel direction. Setting yaw_angle to zero with
        face_travel_direction == True will results in object always facing its
        travel direction. Non-zero yaw_angle will cause additional yaw offsets.
      repeat_mode: Behavior of object after reaching the last way point in list.
        If the value is Text, it is converted to PatrolRepeatMode.
      speed_mps: Speed in meters per second.
      time_points: List of times associated with points. These times
        represent when the object should arrive at the associated waypoint.
        Optional, but if provided it must have the same length as 'points'.
        If 'speed_mps' is None, then 'time_points' will be used as-is and there
        is no maximum segment speed. If 'speed_mps' is also defined, then it
        serves as a maximum speed value and time points which would result in a
        segment speed above this value will be altered such that the maximum
        segment speed is 'speed_mps'.
    """
    self._repeat = (repeat_mode if isinstance(repeat_mode, PatrolRepeatMode)
                    else PatrolRepeatMode[repeat_mode])
    self._yaw_angle = yaw_angle
    self._face_travel_direction = face_travel_direction
    self._speed_mps = speed_mps

    if len(points) < 2:
      raise ValueError(
          f"Need at least two points in 'points', got {len(points)}")

    if time_points is not None and self._repeat is PatrolRepeatMode.LOOP:
      raise ValueError("Time points are not compatible with LOOP mode.")

    if (self._repeat is PatrolRepeatMode.NO_REPEAT or
        self._repeat is PatrolRepeatMode.RESET):
      augmented_points = points

      if time_points is not None:
        augmented_time_points = time_points
    elif self._repeat is PatrolRepeatMode.LOOP:
      augmented_points = list(points) + [points[0]]
    elif self._repeat is PatrolRepeatMode.BACK_TRACK:
      augmented_points = list(points) + list(reversed(points[:-1]))

      if time_points is not None:
        # Time strictly increases, so add the timepoints again on top
        # of the last entry. We add the difference between the last time
        # and the previous elements (in reverse order), on top of the last
        # element where we left off.
        augmented_time_points = list(time_points) + \
            list(time_points[-1] + (time_points[-1] - np.array(time_points[1::-1])))
    else:
      raise NotImplementedError(
          f"Repeat mode {self._repeat} is not supported yet.")

    augmented_points = np.array(augmented_points)
    # For Nx2 inputs, pad it to Nx3 with default z value of 0.
    if augmented_points.shape[1] == 2:
      augmented_points = np.hstack(
          augmented_points, np.zeros(augmented_points.shape[0], 1))
    elif augmented_points.shape[1] != 3:
      raise ValueError("Expect 'points' to be Nx2 or Nx3.")

    t = 0
    self._segments = []

    if time_points is None:
      for from_point, to_point in zip(
          augmented_points[:-1], augmented_points[1:]):
        segment, t = self._get_patrol_segment_by_speed(from_point, to_point, t)
        self._segments.append(segment)
    else:
      for from_point, to_point, from_time, to_time in zip(
          augmented_points[:-1], augmented_points[1:],
          augmented_time_points[:-1], augmented_time_points[1:]):
        segment, t = self._get_patrol_segment_by_time(from_point, to_point, t,
                                                      t + (to_time - from_time))
        self._segments.append(segment)

    self._segment_times = [l.start_time for l in self._segments]

    self._cycle_time = t

  def _get_patrol_segment_by_time(self,
                                  from_point,
                                  to_point,
                                  from_time,
                                  to_time):
    """Returns a PatrolSegmentData for the given points and times."""
    unit_vector, length = self._get_vector(from_point, to_point)
    orientation = self._get_orientation(unit_vector)

    if np.isclose(to_time, from_time):
      speed_mps = 0
    else:
      speed_mps = length / (to_time - from_time)

      if self._speed_mps is not None:
        speed_mps = np.min([self._speed_mps, speed_mps])

    if np.isclose(0, speed_mps):
      new_to_time = to_time
    else:
      new_to_time = np.max([to_time, from_time + (length / speed_mps)])

    segment = PatrolSegmentData(
        start_time=from_time,
        start_position=np.array(from_point),
        velocity=unit_vector * speed_mps,
        orientation=orientation)

    return segment, new_to_time

  def _get_patrol_segment_by_speed(self, from_point, to_point, current_time):
    """Returns a PatrolSegmentData for the given points and a constant speed."""
    unit_vector, length = self._get_vector(from_point, to_point)
    orientation = self._get_orientation(unit_vector)

    segment = PatrolSegmentData(
        start_time=current_time,
        start_position=np.array(from_point),
        velocity=unit_vector * self._speed_mps,
        orientation=orientation)
    time = current_time + length / self._speed_mps

    return segment, time

  def _get_vector(self, from_point, to_point):
    """Gets the unit vector and length of a from/to point pair."""
    vector = np.array(to_point) - np.array(from_point)
    length = np.linalg.norm(vector)

    if length == 0:
      raise ValueError(f"Length of patrol segment equal to 0, "
                       f"from {from_point} to {to_point}.")
    unit_vector = vector / length

    return unit_vector, length

  def _get_orientation(self, unit_vector):
    """Gets the orientation quaternion given a unit vector."""
    yaw = (np.arctan2(unit_vector[1], unit_vector[0])
           if self._face_travel_direction else 0) + self._yaw_angle

    orientation = np.array((0, 0, np.sin(yaw / 2), np.cos(yaw / 2)))

    return orientation

  def get_action(self,
                 t: float,
                 observations: Dict[Text, Any]) -> ControllerOutput:
    """Returns position on, and orientation along the patrol segment."""
    del observations

    # t < 0 means initial condition, which is the same as the value at t = 0.
    t = max(0, t)

    if t > self._cycle_time:
      t = (self._cycle_time if self._repeat is PatrolRepeatMode.NO_REPEAT
           else np.fmod(t, self._cycle_time))

    segment = self._segments[bisect.bisect_right(self._segment_times, t) - 1]
    position = (
        segment.start_position + segment.velocity * (t - segment.start_time))

    return position, segment.orientation.copy(), {}


@gin.configurable
class LinearPatrolController(WayPointPatrolController):
  """Controller for patrolling along a line segment (back and forth)."""

  def __init__(self,
               from_point: Sequence[float],
               to_point: Sequence[float],
               **kwargs):
    """Constructor.

    Args:
      from_point: Starting point of motion, [x, y, z] in meters.
      to_point: Returning point of motion, [x, y, z] in meters.
      **kwargs: Keyword arguments to pass onto base class.
    """
    super().__init__([from_point, to_point],
                     repeat_mode=PatrolRepeatMode.LOOP,
                     **kwargs)


# TODO(b/156126975): migrates this to use difference equation controller.
@gin.configurable
class ChaseController(ControllerBase):
  """Controller for an object to chase another object at certain speed."""

  def __init__(self,
               self_key: Text,
               target_key: Text,
               initial_position: Sequence[float] = (0, 0, 0),
               initial_orientation: Sequence[float] = (0, 0, 0, 1),
               speed_mps: float = 1.0,
               verbose: bool = False):
    """Constructor.

    Args:
      self_key: Observation dict key of position of object being controlled.
      target_key: Observation dict key of position of target object.
      initial_position: Initial position of the object.
      initial_orientation: Initial orientation of the object in xyzw quaternion.
      speed_mps: Speed in meters per second, always positive.
      verbose: If True, log details of get_action() calculation for debugging.
    """
    self._init_position = np.array(initial_position)
    self._init_orientation = np.array(initial_orientation)
    self._previous_orientation = self._init_orientation
    if speed_mps <= 0:
      raise ValueError(
          f"'speed_mps' should be a positive value, got {speed_mps}.")
    self._speed_mps = speed_mps

    self._self_key = self_key
    self._target_key = target_key
    self._verbose = verbose

    self._time_sec = 0

  def get_action(self,
                 time_sec: float,
                 observations: Dict[Text, Any]) -> ControllerOutput:
    """Returns position and orientation of the object being controlled.

    Args:
      time_sec: Time since simulation reset in seconds. If time < 0, returns
        initial values and ignores observations.
      observations: A dict of all observations.
    """
    if time_sec < 0:
      # Initializes internal time.
      self._time_sec = 0
      return self._init_position.copy(), self._init_orientation.copy(), {}

    self_position = observations[self._self_key]
    target_position = observations[self._target_key]

    # Calculates delta vector and projects it to xy-plane.
    delta_vector = (target_position - self_position) * (1, 1, 0)
    delta_t = time_sec - self._time_sec

    # Advances internal time.
    self._time_sec = time_sec

    if self._verbose:
      with np.printoptions(precision=3, suppress=True):
        logging.info("t = %.1f, self %s: %s, target %s: %s, v: %s, dt %.1f.",
                     self._t,
                     self._self_key, observations[self._self_key],
                     self._target_key, observations[self._target_key],
                     delta_vector, delta_t)

    # Avoids sigularity when it is close enough. Keeps previous orientation.
    distance = np.linalg.norm(delta_vector)
    if distance < _EPS_DISTANCE:
      return target_position.copy(), self._previous_orientation.copy(), {}

    unit_delta_vector = delta_vector / distance
    new_position = (unit_delta_vector * min(self._speed_mps * delta_t, distance)
                    + self_position)

    new_yaw = np.arctan2(unit_delta_vector[1], unit_delta_vector[0])
    new_orientation = np.array((0, 0, np.sin(new_yaw / 2), np.cos(new_yaw / 2)))
    self._previous_orientation = new_orientation

    return new_position, new_orientation.copy(), {}


@gin.configurable
class AnimationFrameController(ControllerBase):
  """An extra action controller to control playback of animation sequence."""

  def __init__(self, fps: float = 10.0,
               pause_between_repeat_sec: float = 0.0,
               start_time_sec: float = 0.0):
    """Constructor.

    Args:
      fps: Frame per second of animation.
      pause_between_repeat_sec: Pause between repeat in second.
      start_time_sec: The time when animation starts to play.
    """
    self._fps = fps
    self._total_length = None
    self._pause_between_repeat_sec = pause_between_repeat_sec
    self._start_time_sec = start_time_sec

  def set_total_length(self, total_length: int):
    """Sets total animation frame length."""
    if total_length <= 0:
      raise ValueError(
          f"Total number of frame must be >= 0, got {total_length}.")
    self._total_length = total_length

  def get_action(self,
                 time_sec: float,
                 observations: Dict[Text, Any]) -> ControllerOutput:
    """Returns animation frame number with default position and orientation.

    Args:
      time_sec: Time since simulation reset in seconds. If time < 0, returns
        initial values and ignores observations.
      observations: A dict of all observations.
    """
    time_sec = max(0, time_sec - self._start_time_sec)
    frame = int(time_sec * self._fps)
    if self._total_length:
      frame = frame % (
          self._total_length + int(self._pause_between_repeat_sec * self._fps))
      frame = min(frame, self._total_length - 1)
    return np.ndarray((0, 0, 0)), np.ndarray((0, 0, 0, 1)), {
        ANIMATION_FRAME_NUMBER_KEY: frame}


@gin.configurable
class ConversationController(ControllerBase):
  """Controller for an object that mimics conversational behavior.

  A controlled object is arrayed in a conversation about a center point.
  When a target object reaches a thresholded distance away from the center,
  the controlled object will face the target object and move away from
  the target's intended path along an orthogonal direction vector until it
  passes.
  """

  def __init__(self,
               self_key: Text,
               target_key: Text,
               position: Sequence[float] = None,
               orientation: Sequence[float] = None,
               conversation_center: Sequence[float] = None,
               proximity_threshold: float = 1.0,
               speed_mps: float = 0.1):
    """Constructor.

    Args:
      self_key: Observation dict key of position of object being controlled.
      target_key: Observation dict key of position of target object.
      position: Initial position of the object.
      orientation: Initial orientation of the object in xyzw quaternion.
      conversation_center: Position of the center of the conversation group.
      proximity_threshold: The distance from the conversation center that the
        target must reach in order to prompt a response from the controlled
        object.
      speed_mps: Speed in meters per second, always positive.
    """
    self._self_key = self_key
    self._target_key = target_key
    self._position = np.array(position or (0, 0, 0))
    self._orientation = np.array(orientation or (0, 0, 0, 1))
    self._conversation_center = np.array(conversation_center or (0, 0, 0))
    self._proximity_threshold = proximity_threshold
    self._speed_mps = speed_mps

    self._prev_target_position = None
    self._wait_position = None

  def _get_wait_position(self, self_position, target_position):
    """Gets the waiting position for the controlled object.

    This returns the position that the controlled object should move towards
    to create physical space such that the target object may pass through
    the conversation space.

    Args:
      self_position: The current position of the controlled object.
      target_position: The current position of the target object.

    Returns:
      An xyz position representing the waiting position that the controlled
      object should move towards to create space.
    """
    # Find an orthogonal projection to the target's path
    target_path = np.array(target_position - self._prev_target_position)
    self_path = np.array(self_position - self._prev_target_position)

    unit_target_path = target_path / np.linalg.norm(target_path)

    projected_point = np.dot(self_path, unit_target_path) * unit_target_path
    projected_point += self._prev_target_position

    # Get the position that lies along the orthogonal projection vector
    # but in the opposite direction from the target's path and exactly
    # proximity_threshold distance away.
    return self._get_position(
        projected_point,
        self_position,
        self._proximity_threshold,
        self._proximity_threshold)

  def _get_orientation(self, source_position, target_position):
    """Gets orientation required to face target_position from source_position.

    Args:
      source_position: The source position where an object would be located.
      target_position: The target position that an object should face.

    Returns:
      A xyzw quaternion indicating the orientation.
    """
    if np.allclose(source_position, target_position):
      return self._orientation

    delta_vector = (target_position - source_position) * (1, 1, 0)

    new_yaw = np.arctan2(delta_vector[1], delta_vector[0])
    new_orientation = np.array(
        (0, 0, np.sin(new_yaw / 2), np.cos(new_yaw / 2)))

    return new_orientation

  def _get_position(self,
                    source_position,
                    target_position,
                    min_delta,
                    max_delta):
    """Gets the next position along the vector from source to target.

    This returns the position that should be moved to next which lies along
    the direction vector from source -> target with a minimum length of
    min_delta and a maximum distance of max_delta.

    Args:
      source_position: The current position of the controlled object.
      target_position: The target position to move to.
      min_delta: The minimum amount of distance to move.
      max_delta: The maximum amount of distance to move.

    Returns:
      An xyz position representing the next position to move to.
    """
    delta_vector = (target_position - source_position) * (1, 1, 0)
    distance = np.linalg.norm(delta_vector)

    # If the distance to the target is greater than the maximum step delta,
    # then normalize the vector and set it to the max step delta.
    if distance > max_delta:
      delta_vector = (delta_vector / distance) * max_delta
    # If the distance is less than the minimum step delta, then normalize
    # the vector and set it to the min step delta.
    elif distance < min_delta:
      delta_vector = (delta_vector / distance) * min_delta

    new_position = (delta_vector + source_position)

    return new_position

  def _get_target_distance_to_center(self, target_position):
    """Gets the distance from the target to the conversation center point.

    Args:
      target_position: The target position.

    Returns:
      The scalar distance from the target position to the conversation center.
    """
    # Calculates delta vector and projects it to xy-plane.
    delta_vector = (target_position - self._conversation_center) * (1, 1, 0)

    # Compute the length of the delta vector.
    return np.linalg.norm(delta_vector)

  def get_action(self,
                 t: float,
                 observations: Dict[Text, Any]) -> ControllerOutput:
    """Gets the position and orientation of the controlled object.

    Args:
      t: The current time step.
      observations: Dict containing sensor observations for current time step.

    Returns:
      The new position and orientation for the controlled object.
    """

    position = self._position
    orientation = self._orientation

    # Observations are only available for positive time steps.
    if t >= 0:
      self_position = observations[self._self_key]
      target_position = observations[self._target_key]

      target_distance = self._get_target_distance_to_center(target_position)

      # Check if the target is within the threshold distance of the
      # conversation center.
      if(target_distance < self._proximity_threshold and
         self._prev_target_position is not None):

        # If it is, get the position that the controlled object should move to
        # in order to create space and get the resulting action/orientation.
        wait_position = self._get_wait_position(self_position, target_position)

        orientation = self._get_orientation(
            self_position,
            target_position)

        position = self._get_position(
            self_position,
            wait_position,
            0.0,
            self._speed_mps)
      else:
        # Otherwise, get the position/orientation required to move back to
        # the original position and face the conversation center.
        orientation = self._get_orientation(
            self_position,
            self._conversation_center)

        position = self._get_position(
            self_position,
            self._position,
            0.0,
            self._speed_mps)

      self._prev_target_position = target_position

    # Only return a new position along the x/y axis, z should be unaffected.
    position = np.array([position[0], position[1], self._position[2]])
    return position, orientation, {}


@gin.configurable
class PauseIfCloseByWrapper(ControllerBase):
  """A controller wrapper that pauses controller if object is close to others.

  This wrapper works best if the underlying controller is time based. It is
  intended to be a simple way to stop agent when blocked and is not for
  reliable collision avoidance.
  """

  _DEFAULT_PAUSE_DISTANCE_M = 1.0

  def __init__(
      self,
      wrapped_controller: ControllerBase,
      self_pos_key: Text,
      others_pos_keys: Sequence[Text],
      pause_distance: Union[float, Sequence[float]] = _DEFAULT_PAUSE_DISTANCE_M,
      self_yaw_key: Optional[Text] = None,
      active_front_sector: Optional[float] = None):
    """Constructor.

    Args:
      wrapped_controller: The controller being wrapped.
      self_pos_key: Observation key of self position.
      others_pos_keys: Observation keys of others' positions.
      pause_distance: The distance limit before the controller pauses in meters.
        Can be a float value which applies to all objects specified in
        others_pos_keys or a Sequence of float values with the same length
        as others_pos_keys denoting the pause distance for each individual
        object in the same order in others_pos_keys. Default pause distance is
        one meter.
      self_yaw_key: Observation key of self yaw. Required if active_front_sector
        is specified.
      active_front_sector: If specified, it defines pie-shaped active region in
        front of controlled object. The pie-shaped area is symmetric about the
        forward direction of controlled object with it angle defined by this
        arg in radians, Only when other objects shows up in this region and
        pause distance requirement is met, pause is actived.
    """

    self._controller = wrapped_controller
    self._pause_start_t = -1
    self._shift_t = 0
    self._last_action = None
    self._self_pos_key = self_pos_key
    self._others_pos_keys = list(others_pos_keys)  # Make a copy.

    if isinstance(pause_distance, float):
      pause_distance = [pause_distance] * len(others_pos_keys)

    if len(pause_distance) != len(others_pos_keys):
      raise ValueError(
          "pause_distance and others_pos_keys must have the same length.")
    self._pause_distance = list(pause_distance)  # Make a copy.

    if active_front_sector is not None and self_yaw_key is None:
      raise ValueError(
          "self_yaw_key must be specified if active_front_sector is specified.")

    self._self_yaw_key = self_yaw_key
    self._active_front_sector = active_front_sector

  def get_action(self,
                 t: float,
                 observations: Dict[Text, Any]) -> ControllerOutput:

    """Gets the position and orientation of the controlled object.

    Args:
      t: The current time step.
      observations: Dict containing sensor observations for current time step.

    Returns:
      The new position and orientation for the controlled object.
    """
    if t < 0:
      self._pause_start_t = -1
      self._shift_t = 0
      self._last_action = self._controller.get_action(
          t, observations)
      return self._last_action

    if self._should_pause(observations):
      # Only record the start time of pause.
      if self._pause_start_t < 0:
        self._pause_start_t = t
      return self._last_action
    else:
      if self._pause_start_t >= 0:
        self._shift_t += t - self._pause_start_t
        self._pause_start_t = -1

      self._last_action = self._controller.get_action(
          t - self._shift_t, observations)
      return self._last_action

  def _should_pause(self, observations) -> bool:
    """Determines whether the controller should pause."""
    self_position_2d = observations[self._self_pos_key][:2]
    for pos_key, pause_distance in zip(
        self._others_pos_keys, self._pause_distance):
      position_2d = observations[pos_key][:2]
      vector_2d = position_2d - self_position_2d
      distance = np.linalg.norm(vector_2d)

      if self._active_front_sector is None:
        return distance <= pause_distance
      else:
        yaw = observations[self._self_yaw_key][0]
        dot = np.dot(vector_2d / distance, np.array([np.cos(yaw), np.sin(yaw)]))
        return (distance <= pause_distance and
                np.arccos(dot) < self._active_front_sector / 2)

    return False
