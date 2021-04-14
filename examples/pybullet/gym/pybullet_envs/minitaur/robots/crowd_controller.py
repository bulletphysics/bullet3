# Lint as: python3
"""Crowd objects/human controllers module."""

import abc
import collections
from typing import Any, Callable, Dict, Iterable, List, Optional, Union, Sequence, Text

from absl import logging
import dataclasses
import gin
import numpy as np
#import rvo2

from pybullet_envs.minitaur.envs_v2.sensors import base_position_sensor
from pybullet_envs.minitaur.envs_v2.sensors import sensor as generic_sensor
from pybullet_envs.minitaur.robots import autonomous_object
from pybullet_envs.minitaur.robots import object_controller


POSITION_SENSOR_POSTFIX = "_pos"


@dataclasses.dataclass
class MovingObjectRecord:
  position_key: Text
  agent_id: int
  radius: float
  last_position: Optional[np.ndarray] = None


@gin.configurable
def sample_start_target_position(scene,
                                 start=None,
                                 start_circles=None,
                                 target_circles=None,
                                 num_sampling_retries=1,
                                 min_wall_distance=0.0,
                                 min_goal_euclidean_distance=0.0,
                                 max_goal_euclidean_distance=np.Inf,
                                 min_path_clearance=None):
  """Sample valid start and target position reachable from start.

  Args:
    scene: a SceneBase instance implementing get_random_valid_position function.
    start: a 2-tuple (x, y) of start position. If specified, no start is
      sampled.
    start_circles: a list of circle specification. Each circle is specified as
      a tuple ((x, y), r) of a center (x, y) and radius r. If specified, start
      position is sampled from within one of the start_circles.
    target_circles: same as start_circle. If specified, target positions is
      sampled from within one of the start_circles.
    num_sampling_retries: a positive int, number of attempts to sample a
      start, target pair.
    min_wall_distance: a float, the minimum distance to a wall.
    min_goal_euclidean_distance: a positive float, the minimum distance between
      start and target.
    max_goal_euclidean_distance: a positive float, the maximum distance between
      start and target.
    min_path_clearance: float, clearance of shortest path to walls.

  Returns:
    A 4 tuple (start, target, shortest_path, is_valid). start and target are
    start and target positions, shortest_path is a list of 2-tuples specifying
    the shortest path from start to target, is_valid is bool specifying whether
    the start, target pair is valid. If min_path_clearance is not specified,
    then shortest_path is None.
  """
  if not hasattr(scene, "get_random_valid_position"):
    raise ValueError(
        "Incompatible scene {}. Expected to have `get_random_valid_position` "
        "method.".format(scene))

  def _print_counters(counters):
    for name, value in counters.items():
      logging.info("  %s: %d", name, value)

  sampling_counters = collections.defaultdict(lambda: 0)
  for _ in range(num_sampling_retries):
    if start is None:
      start_pos = scene.get_random_valid_position(
          min_wall_distance, inclusion_circles=start_circles)
    else:
      if start_circles is not None:
        raise ValueError("At most one of the arguments start and start_circles "
                         "can be not None.")
      start_pos = start
    target_pos = scene.get_random_valid_position(
        min_wall_distance, inclusion_circles=target_circles)
    sampling_counters["attempts"] += 1

    euclidean_distance = np.linalg.norm(target_pos - start_pos)
    if euclidean_distance < min_goal_euclidean_distance:
      sampling_counters["min_euclidean"] += 1
      continue
    if euclidean_distance > max_goal_euclidean_distance:
      sampling_counters["max_euclidean"] += 1
      continue

    # Skip the path computation is no path clearance is provided.
    if min_path_clearance is None:
      logging.info("Valid goal with no minimum path clearance checking.")
      _print_counters(sampling_counters)
      return start_pos, target_pos, None, True

    # Check the goal clearance along the shortest path
    if not hasattr(scene, "find_shortest_path"):
      raise ValueError(
          f"scene %s missing find_shortest_path method {scene}")

    # This is a slow process.
    shortest_path = scene.find_shortest_path(
        start_pos[:2], target_pos[:2], min_path_clearance)
    # No path exists between current robot position and goal satisfying the
    # clearance.
    if shortest_path is None:
      sampling_counters["path_clearance"] += 1
      continue

    logging.info("Valid start/target with path clearance checking.")
    _print_counters(sampling_counters)
    return start_pos, target_pos, shortest_path, True

  logging.info("No valid start/target found.")
  _print_counters(sampling_counters)
  return start_pos, target_pos, None, False


class CrowdController(metaclass=abc.ABCMeta):
  """Crowd controller interface."""

  def __init__(self, names: Iterable[Text],
               position_key_formatter="%s" + POSITION_SENSOR_POSTFIX):
    """Constructor.

    Args:
      names: Name of instance (dynamic object or human).
      position_key_formatter: Formatter to convert name to position sensor name.
    """
    self._names = list(names)
    self._position_key_formatter = position_key_formatter
    self._num_instance = len(self._names)

    self._current_time = 0

  def _validate_instance_id(self, instance_id):
    if not 0 <= instance_id < self._num_instance:
      raise ValueError(
          f"instance_id must be an integer in [0, {self.num_instance}), "
          f"got {instance_id}.")

  @property
  def num_instance(self):
    """Returns the number of crowd instances."""
    return self._num_instance

  def instance_name(self, instance_id: int) -> Text:
    """Returns the name of instance."""
    self._validate_instance_id(instance_id)
    return self._names[instance_id]

  def instance_controller(
      self, instance_id: int) -> object_controller.ControllerBase:
    """Returns the individual controller of certain instance."""
    self._validate_instance_id(instance_id)
    return _IndividualController(self, instance_id)

  def instance_get_action(
      self, instance_id: int, time_sec: float,
      observations: Dict[Text, Any]) -> object_controller.ControllerOutput:
    """Returns action of specific instance given observation.

    This method is for _IndividualController.

    Args:
      instance_id: Identifier of an object in the crowd.
      time_sec: Time since simulation reset in seconds. If time < 0, returns
        initial values and ignores observations.
      observations: A dict of all observations.

    Returns:
      Position, orientation and an extra info dict for robot joints, human
        skeletal pose, etc.
    """
    if time_sec < 0:
      self._recalculate_actions(object_controller.INIT_TIME, {})
      self._current_time = object_controller.INIT_TIME
    elif time_sec > self._current_time:
      self._current_time = time_sec
      self._recalculate_actions(self._current_time, observations)

    self._validate_instance_id(instance_id)

    return self._get_action_of_instance(instance_id)

  @abc.abstractmethod
  def _recalculate_actions(
      self, time_sec: float, observations: Dict[Text, Any]) -> None:
    """Calculates crowd command for all instances in crowd."""
    raise NotImplementedError(
        "_recalculate_actions() should be implemented by subclass.")

  @abc.abstractmethod
  def _get_action_of_instance(
      self, instance_id: int) -> object_controller.ControllerOutput:
    """Returns calculated actions of specific instance."""
    raise NotImplementedError(
        "_get_action_of_instance() should be implemented by subclass.")

  def set_scene(self, scene) -> None:
    """Sets the scene for crowd controller to obtain scene information."""
    del scene


class _IndividualController(object_controller.ControllerBase):
  """A utility class that wraps crowd controller in ControllerBase interface."""

  def __init__(self, crowd_controller: CrowdController, instance_id: int):
    """Constructor.

    Args:
      crowd_controller: The controller of crowd to which this instance belong.
      instance_id: Identifier of a crowd instance.
    """
    self._instance_id = instance_id
    self._crowd_controller = crowd_controller

  def get_action(
      self, time_sec: float,
      observations: Dict[Text, Any]) -> object_controller.ControllerOutput:
    """Returns position, orientation and pose based on time and observations.

    Args:
      time_sec: Time since simulation reset in seconds. If time < 0, returns
        initial values and ignores observations.
      observations: A dict of all observations.

    Returns:
      Position, orientation and an extra info dict for robot joints, human
        skeletal pose, etc.
    """
    return self._crowd_controller.instance_get_action(
        self._instance_id, time_sec, observations)


@gin.configurable
class StationaryController(CrowdController):
  """A crowd controller that places crowd objects at fixed positions."""

  def __init__(
      self, positions: Sequence[Sequence[float]],
      orientations: Optional[Sequence[Sequence[float]]] = None, **kwargs):
    """Constructor.

    Args:
      positions: Fixed positions (3D points) of crowd instances.
      orientations: Fixed orientations in quaternion of crowd instances.
      **kwargs: Keyword arguments to pass on to base class.
    """
    super().__init__(**kwargs)

    if orientations is None:
      orientations = np.array(((0, 0, 0, 1),) * self.num_instance)

    if not len(positions) == len(orientations) == self.num_instance:
      raise ValueError(
          f"positions and orientations should all have the same length "
          f"{self.num_instance}. Got len(positions) = {len(positions)}, "
          f"len(orientations) = {len(orientations)}.")

    self._positions = positions
    self._orientations = orientations

  def _recalculate_actions(
      self, time_sec: float, observations: Dict[Text, Any]) -> None:
    """Calculates crowd command for all instances in crowd."""
    del time_sec
    del observations

  def _get_action_of_instance(
      self, instance_id: int) -> object_controller.ControllerOutput:
    """Returns calculated actions of specific instance."""
    self._validate_instance_id(instance_id)
    return self._positions[instance_id], self._orientations[instance_id], {}


@gin.configurable
class OrcaController(CrowdController):
  """A crowd controller that controls crowd instances using ORCA algorithm.

  Crowd instance will be initialized at a specified start position and move
  towards specified target position in a linear path while avoid collision with
  each other.
  """

  _DEFAULT_NEIGHBOR_DISTANCE_M = 5
  _DEFAULT_MAX_NEIGHBORS = 10
  _DEFAULT_RADIUS_M = 0.5
  _DEFAULT_MAX_SPEED_MPS = 2
  _DEFAULT_TIME_HORIZON_SEC = 1.0
  _DEFAULT_OBSTACLE_TIME_HORIZON_SEC = 0.3

  def __init__(
      self,
      timestep: float,
      start_positions: Optional[Sequence[Sequence[float]]] = None,
      target_positions: Optional[Sequence[Sequence[float]]] = None,
      use_position_generator: Optional[bool] = False,
      group_sizes: Sequence[int] = None,
      radius: float = _DEFAULT_RADIUS_M,
      max_speed_mps: float = _DEFAULT_MAX_SPEED_MPS,
      time_horizon_sec: float = _DEFAULT_TIME_HORIZON_SEC,
      obstacle_time_horizon_sec: float = _DEFAULT_OBSTACLE_TIME_HORIZON_SEC,
      neighbor_distance_m: float = _DEFAULT_NEIGHBOR_DISTANCE_M,
      max_neighbors: int = _DEFAULT_MAX_NEIGHBORS,
      workaround_erp_issue: bool = True,
      moving_objects_pos_key: Sequence[Text] = (),
      moving_objects_radius: Union[float, Sequence[float]] = _DEFAULT_RADIUS_M,
      endless_trajectory: bool = True,
      **kwargs):
    """Constructor.

    Args:
      timestep: Timestep of simulation.
      start_positions: A list of position (x, y, z) for crowd instances as
        their starting position.
      target_positions: A list of position (x, y, z) for crowd instances as
        their target position.
      use_position_generator: a boolean, if True than the start and end
        positions are sampled. start_positions and target_positions must be None
      group_sizes: If set, then crowd is split in groups randomly, whose sizes
        are picked in random from this group_size list. In this way, the
        crowd simulator sumulaters clusters of objects moving around.
      radius: Radius of crowd instances.
      max_speed_mps: Maximum crowd instance speed.
      time_horizon_sec: Time horizon in second.
      obstacle_time_horizon_sec: Time horizon for static obstacle in second.
      neighbor_distance_m: Neighbor distance in meters. Instances closer than
        this distance are considered neighbors.
      max_neighbors: Max number of neighbors.
      workaround_erp_issue: There is an issue with pybullet constraint that the
        constraint is solved only 20% per timestep. Need to amplify position
        delta by 5x to workaround this issue.
      moving_objects_pos_key: Position observation key of moving objects not
        controlled by the ORCA controller.
      moving_objects_radius: Radius of moving objects. Should be a float, which
        applies to all moving objects, or a sequence of float, which should be
        of the same length as moving_objects_pos_key.
      endless_trajectory: Only valid if use_position_generator is True. Agent
        returns to starting point after reaching goal to achieve endless motion.
      **kwargs: Keyword arguments to pass on to base class.
    """
    super().__init__(**kwargs)

    assert ((start_positions is not None and target_positions is not None) or
            use_position_generator)
    if not use_position_generator:
      if not len(start_positions) == len(target_positions) == self.num_instance:
        raise ValueError(
            f"start_positions and target_positions should both have length "
            f"equals {self.num_instance}: "
            f"len(start_positions) = {len(start_positions)}, "
            f"len(target_positions) = {len(target_positions)}.")

    self._timestep = timestep
    self._radius = radius
    self._max_speed_mps = max_speed_mps
    self._time_horizon_sec = time_horizon_sec
    self._obstacle_time_horizon_sec = obstacle_time_horizon_sec
    self._neighbor_distance_m = neighbor_distance_m
    self._max_neighbors = max_neighbors
    self._use_position_generator = use_position_generator
    self._endless_trajectory = endless_trajectory
    self._scene = None
    if isinstance(moving_objects_radius, float):
      moving_objects_radius = [
          moving_objects_radius] * len(moving_objects_pos_key)
    if len(moving_objects_radius) != len(moving_objects_pos_key):
      raise ValueError(
          "moving_objects_radius should be either a float or a sequence of "
          "float with the same length as moving_objects_pos_key.")
    self._moving_objects = [
        MovingObjectRecord(position_key=key, agent_id=-1, radius=radius)
        for key, radius in zip(moving_objects_pos_key, moving_objects_radius)]

    self._paths = None
    self._path_indices = None
    if self._use_position_generator:
      self._start_positions = None
      self._target_positions = None
    else:
      self._start_positions = np.array(start_positions, dtype=np.float64)
      self._target_positions = np.array(target_positions, dtype=np.float64)
    # A guard against multiple initializations. See recalculate_actions below.
    self._already_initialized = False
    self._group_sizes = [1] if group_sizes is None else group_sizes

    # The following variables are initialized in _recalculate_actions()
    self._current_positions = None
    self._command_positions = None
    self._command_orientations = None

    #self._orca = rvo2.PyRVOSimulator(
    #    self._timestep,  # timestep
    #    self._neighbor_distance_m,  # neighborDist
    #    self._max_neighbors,  # maxNeighbors
    #    self._time_horizon_sec,  # timeHorizon
    #    self._obstacle_time_horizon_sec,  # timeHorizonObst
    #    self._radius,  # radius
    #    self._max_speed_mps  # maxSpeed
    #)
    for i in range(self.num_instance):
      if self._use_position_generator:
        start_position = (0, 0)
      else:
        start_position = self._start_positions[i, :2]
      agent_id = self._orca.addAgent(
          tuple(start_position),
          self._neighbor_distance_m,  # neighborDist
          self._max_neighbors,  # maxNeighbors
          self._time_horizon_sec,  # timeHorizon
          self._obstacle_time_horizon_sec,  # timeHorizonObst
          self._radius,  # radius
          self._max_speed_mps,  # maxSpeed
          (0.0, 0.0))  # velocity
      assert agent_id == i

    for obj in self._moving_objects:
      obj.agent_id = self._orca.addAgent(
          (0.0, 0.0),  # position (will adjust after simulation starts)
          self._neighbor_distance_m,  # neighborDist
          self._max_neighbors,  # maxNeighbors
          self._timestep,  # timeHorizon
          self._timestep,  # timeHorizonObst
          obj.radius,  # radius
          self._max_speed_mps,  # maxSpeed
          (0.0, 0.0))  # velocity

    self._workaround_erp_issue = workaround_erp_issue

  def _subsample_path(self, path, subsample_step=1.0):
    subsampled_path = [path[0]]
    traveled_dist = 0.0
    for i, (s, t) in enumerate(zip(path[:-1], path[1:])):
      traveled_dist += np.sqrt(
          np.square(s[0] - t[0]) + np.square(s[1] - t[1]))
      if traveled_dist > subsample_step or i >= len(path) - 2:
        subsampled_path.append(t)
        traveled_dist = 0.0
    return subsampled_path

  def _generate_start_target_positions(self):
    """Generates start and target positions using goal generartors."""
    assert self._scene is not None
    self._start_positions = np.zeros((self.num_instance, 3), dtype=np.float64)
    self._target_positions = np.zeros((self.num_instance, 3), dtype=np.float64)

    self._paths = []
    self._path_indices = []
    start_circles, target_circles = None, None
    group_radius = 1.0
    current_group_size = np.random.choice(self._group_sizes)
    index_in_current_group = 0
    for i in range(self._num_instance):
      start_pos, target_pos, path, is_valid = sample_start_target_position(
          self._scene,
          start_circles=start_circles,
          target_circles=target_circles)
      if index_in_current_group == current_group_size - 1:
        start_circles, target_circles = None, None
        index_in_current_group = 0
        current_group_size = np.random.choice(self._group_sizes)
      else:
        if start_circles is None:
          start_circles = [(start_pos[:2], group_radius)]
          target_circles = [(target_pos[:2], group_radius)]
        else:
          start_circles += [(start_pos[:2], group_radius)]
          target_circles += [(target_pos[:2], group_radius)]
        index_in_current_group += 1
      if not is_valid:
        raise ValueError("No valid start/target positions.")
      self._start_positions[i, :] = start_pos
      self._target_positions[i, :] = target_pos

      subsampled_path = self._subsample_path(path)
      self._paths.append(np.array(subsampled_path, dtype=np.float32))
      self._path_indices.append(0)

  def _recalculate_actions(
      self, time_sec: float, observations: Dict[Text, Any]) -> None:
    """Calculates crowd command for all crowd instances."""
    if self._use_position_generator:
      if (time_sec == object_controller.INIT_TIME and
          self._start_positions is None and
          not self._already_initialized):
        self._generate_start_target_positions()
        # Initialize only once per initial time even if recalculate actions
        # is called multiple times.
        self._already_initialized = True
    if time_sec == object_controller.INIT_TIME:
      # Resets orca simulator.
      for i in range(len(self._names)):
        self._orca.setAgentPosition(i, tuple(self._start_positions[i, :2]))

      self._command_positions = self._start_positions.copy()
      self._current_positions = self._start_positions.copy()
      self._command_orientations = np.repeat(
          ((0.0, 0.0, 0.0, 1.0),), len(self._names), axis=0)
      self._last_target_recalculation_sec = time_sec
      return
    else:
      # The moment we step beyond initial time, we can initialize again.
      self._already_initialized = False

    if self._use_position_generator:
      for i in range(self._num_instance):
        dist = np.linalg.norm(
            self._current_positions[i, :] - self._target_positions[i, :])
        if dist < 2.0:
          _, target_pos, path, is_valid = sample_start_target_position(
              self._scene, self._current_positions[i, :])
          if is_valid:
            self._target_positions[i, :] = target_pos
            subsampled_path = self._subsample_path(path)
            self._paths.append(np.array(subsampled_path, dtype=np.float32))
            self._path_indices.append(0)

    # Sets agent position and preferred velocity based on target.
    for i, agent_name in enumerate(self._names):
      position = observations[self._position_key_formatter % agent_name]
      self._orca.setAgentPosition(
          i, tuple(position[:2]))  # ORCA uses 2D position.
      self._current_positions[i, :2] = position[:2]

      if self._paths is not None:
        # Find closest point on the path from start to target, which (1) hasn't
        # been covered already; (2) is at least max_coverage_distance away from
        # current position.
        distances = np.sqrt(np.sum(np.square(
            self._paths[i] - position[:2]), axis=1))
        max_coverage_distance = 1.0
        index = self._path_indices[i]
        while True:
          if index >= len(self._paths[i]) - 1:
            if self._endless_trajectory:
              self._paths[i] = self._paths[i][::-1]
              distances = distances[::-1]
              index = 0
            break
          elif distances[index] > max_coverage_distance:
            break
          else:
            index += 1
        self._path_indices[i] = index
        target_position = self._paths[i][index, :]
      else:
        target_position = self._target_positions[i][:2]

      goal_vector = target_position - position[:2]
      goal_vector_norm = np.linalg.norm(goal_vector) + np.finfo(np.float32).eps
      goal_unit_vector = goal_vector / goal_vector_norm

      kv = 1
      velocity = min(kv * goal_vector_norm,
                     self._DEFAULT_MAX_SPEED_MPS) * goal_unit_vector
      self._orca.setAgentPrefVelocity(i, tuple(velocity))

    for obj in self._moving_objects:
      position = observations[obj.position_key]
      self._orca.setAgentPosition(obj.agent_id, tuple(position[:2]))
      if obj.last_position is None:
        self._orca.setAgentPrefVelocity(obj.agent_id, (0.0, 0.0))
      else:
        velocity = (position - obj.last_position) / self._timestep
        self._orca.setAgentPrefVelocity(obj.agent_id, tuple(velocity[:2]))
      obj.last_position = position.copy()

    # Advances orca simulator.
    self._orca.doStep()

    # Retrieve agent position and save in buffer.
    for i in range(len(self._names)):
      x, y = self._orca.getAgentPosition(i)
      self._command_positions[i, :2] = (x, y)

      yaw = np.arctan2(y - self._current_positions[i, 1],
                       x - self._current_positions[i, 0])
      self._command_orientations[i] = (0, 0, np.sin(yaw / 2), np.cos(yaw / 2))

  def _get_action_of_instance(
      self, instance_id) -> object_controller.ControllerOutput:
    """Returns calculated actions of specific instance."""

    if self._command_positions is None:
      raise RuntimeError(
          "Attempted to get action of instance before _recalculate_actions().")

    self._validate_instance_id(instance_id)

    if self._workaround_erp_issue:
      k_erp = 1 / 0.2
      delta_position = (
          self._command_positions[instance_id] -
          self._current_positions[instance_id])
      command_position = (
          self._current_positions[instance_id] + k_erp * delta_position)
    else:
      command_position = self._command_positions[instance_id].copy()
    return command_position, self._command_orientations[instance_id], {}

  def set_scene(self, scene) -> None:
    """Sets the scene for crowd controller to obtain scene information."""
    try:
      polygons = scene.vectorized_map
      for polygon in polygons:
        self._orca.addObstacle([tuple(point) for point in polygon])
      self._orca.processObstacles()
      self._scene = scene
    except NotImplementedError:
      logging.exception("Scene does not implement vectorized_map property. "
                        "Crowd agent cannot avoid static obstacles.")


@gin.configurable
def uniform_object_factory(
    instance_id: int,
    object_factory: Callable[..., autonomous_object.AutonomousObject],
    *args, **kwargs) -> autonomous_object.AutonomousObject:
  """A wrapper that removes instance_id in default crowd object factory."""
  del instance_id
  return object_factory(*args, **kwargs)


@gin.configurable
def random_object_factory(
    instance_id: int,
    object_factories: Iterable[
        Callable[..., autonomous_object.AutonomousObject]],
    *args, **kwargs) -> autonomous_object.AutonomousObject:
  """A wrapper that removes instance_id in default crowd object factory."""
  del instance_id
  object_factory = np.random.choice(object_factories)
  return object_factory(*args, **kwargs)


@gin.configurable
def sensor_factory(instance_id: int, sensor: Callable[...,
                                                      generic_sensor.Sensor],
                   *args, **kwargs) -> generic_sensor.Sensor:
  del instance_id
  return sensor(*args, **kwargs)


@gin.configurable
class CrowdBuilder(object):
  """A helper class to construct a crowd."""

  def __init__(
      self,
      num_instance: int,
      crowd_controller_factory: Callable[..., CrowdController],
      object_factory: Callable[..., autonomous_object.AutonomousObject],
      sensor_factories: Iterable[Callable[..., generic_sensor.Sensor]] = None):
    """Constructor.

    Args:
      num_instance: Number of autonomous objects in the crowd.
      crowd_controller_factory: A callable that returns a crowd controller
        object.
      object_factory: Callable that returns an autonomous object.
      sensor_factories: list of sensor callables.
    """
    self._objects = []
    crowd_id_prefix = "crowd"
    names = [crowd_id_prefix + "_%d" % i for i in range(num_instance)]

    self._controller = crowd_controller_factory(names=names)

    for i in range(num_instance):
      position_sensor = base_position_sensor.BasePositionSensor(
          name=names[i] + POSITION_SENSOR_POSTFIX)

      # Add additional per agent sensors (e.g. camera, occupancy, etc.).
      add_sensors = []
      if sensor_factories:
        for s in sensor_factories:
          add_sensors.append(
              sensor_factory(
                  instance_id=i, sensor=s, name=names[i] + "_" + s.__name__))

      an_object = object_factory(
          instance_id=i,
          sensors=(position_sensor,) + tuple(add_sensors),
          controller=self._controller.instance_controller(i))

      self._objects.append(an_object)

  @property
  def crowd_objects(self) -> List[autonomous_object.AutonomousObject]:
    """Returns list of AutonomousObjects in the crowd."""
    return self._objects

  @property
  def crowd_controller(self) -> CrowdController:
    """Returns the crowd controller."""
    return self._controller
