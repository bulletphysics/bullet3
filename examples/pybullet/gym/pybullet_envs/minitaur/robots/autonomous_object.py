# Lint as: python3
"""A module that defines autonomous object class and related functions."""
from typing import Any, Callable, Dict, Optional, Sequence, Text, Union

from absl import logging
import gin
import numpy as np

from pybullet_utils import bullet_client
from pybullet_envs.minitaur.envs_v2 import base_client
from pybullet_envs.minitaur.envs_v2.sensors import sensor
from pybullet_envs.minitaur.robots import object_controller
from pybullet_envs.minitaur.robots import robot_base

# The action value to pass into AutonomousObject pre_control_step() and
# apply_action().
AUTONOMOUS_ACTION = None

# Maximum force used in constraint based actuation.
_MAX_FORCE = 1000


# TODO(b/155124699): find a better way actuate object than using constraint or
# modifying URDF.
@gin.configurable
class AutonomousObject(robot_base.RobotBase):
  """Autonomous object that moves/acts in simulation guided by a controller."""

  def __init__(self,
               urdf_file: Text,
               sensors: Sequence[sensor.Sensor] = (),
               controller: object_controller.ControllerBase = None,
               actuate_by_reset: bool = False):
    """Constructor.

    Args:
      urdf_file: The path to urdf file of the object.
      sensors: A list of sensor objects to attach to autonomous object.
      controller: A controller object that governs autonomous object's motion.
        If not specified, StationaryController is used.
      actuate_by_reset: Use pybullet resetBasePositionAndOrientation to actuate
        the object. Default is False, which means actuate by constraint. In the
        actuate by constrained mode, be extra cautious when the position or
        orientation control is based on position or orientation sensor reading
        of the same object. This loop-back condition is known to be problematic
        and causes slower than expected motion.
    """
    self._urdf_file = urdf_file
    self._controller = controller or object_controller.StationaryController()
    self._actuate_by_reset = actuate_by_reset
    self._actuate_function = (
        self._actuate_base_pose
        if not actuate_by_reset else self._reset_base_pose)

    self._sensors = list(sensors)
    self._object_id = -1
    self._constraint_id = -1

    self._pybullet_client = None  # will be initialized in set_sim_client()
    self._clock = None  # will be initialized in set_clock()
    self._init_internal_states()

  def _init_internal_states(self) -> None:
    self._observations_time_since_reset = 0
    self._observations = {}
    self._position = np.zeros(3)
    self._orientation = np.array([0, 0, 0, 1])

  def set_sim_client(self, pybullet_client: bullet_client.BulletClient) -> None:
    """Sets new simulation client and reload assets."""
    self._pybullet_client = pybullet_client
    self._init_internal_states()
    self.load()

  def set_clock(self, clock: Callable[[], float]) -> None:
    """Sets monotonic clock when adding into simulation environment."""
    self._clock = clock

  @property
  def sim_object_id(self):
    return self._object_id

  def update(self, time_since_reset_sec: float,
             observations: Dict[Text, Any]) -> None:
    """Updates simulation time and observations.

    This function should be called before apply_action in each simulation step.

    Args:
      time_since_reset_sec: Time from start of simulation reset in seconds.
      observations: A dict of observations.
    """
    if time_since_reset_sec < self._observations_time_since_reset:
      raise ValueError(
          "Time cannot go backwards. Current t = %f, new t = %f." %
          (self._observations_time_since_reset, time_since_reset_sec))
    self._observations_time_since_reset = time_since_reset_sec
    self._observations = observations

  def _load_urdf(self):
    """Loads object URDF file."""
    try:
      print("loading: ", self._urdf_file)
      self._object_id = self._pybullet_client.loadURDF(self._urdf_file)
    except:
      print("Error: cannot load ", self._urdf_file)
      import sys
      sys.exit(0)

  def load(self) -> None:
    """Reconstructs the robot and resets its states."""
    self._load_urdf()
    if not self._actuate_by_reset:
      self._constraint_id = self._pybullet_client.createConstraint(
          parentBodyUniqueId=self._object_id,
          parentLinkIndex=-1,
          childBodyUniqueId=-1,
          childLinkIndex=-1,
          jointType=self._pybullet_client.JOINT_FIXED,
          jointAxis=(0, 0, 0),
          parentFramePosition=(0, 0, 0),
          childFramePosition=(0, 0, 0),
          childFrameOrientation=(0, 0, 0, 1))

    for s in self._sensors:
      s.set_robot(self)

    # Resets the pose and updates the initial observations.
    self.reset()

  def reset(
      self,
      base_position: Optional[Sequence[float]] = None,
      base_orientation_quaternion: Optional[Sequence[float]] = None,
      controller: Optional[object_controller.ControllerBase] = None) -> None:
    """Resets the states (e.g.

    pose and sensor readings) of the robot.

    This is called at the start of each episode by the environment.

    Args:
      base_position: Robot base position after reset. Must be None.
      base_orientation_quaternion: Robot base orientation after reset. Must be
        None.
      controller: A new controller to replace original controller.
    """
    if base_position is not None or base_orientation_quaternion is not None:
      raise ValueError("Reset position and orientation of AutonomousObject is "
                       "specified in controller.")

    if controller is not None:
      self._controller = controller

    self._init_internal_states()
    self._position, self._orientation, _ = self._controller.get_action(
        object_controller.INIT_TIME, self._observations)

    self._reset_base_pose(self._position, self._orientation)
    self.receive_observation()

  def terminate(self) -> None:
    """Shuts down the robot, no-op in simulation."""

  def pre_control_step(self, action: Any) -> Any:
    """Processes the input action before the action repeat loop.

    Args:
      action: expect it to be `AUTONOMOUS_ACTION` at present.

    Returns:
      the action as is.
    """
    # Environment should not pass action other than AUTONOMOUS_ACTION.
    if action is not AUTONOMOUS_ACTION:
      raise ValueError("AutonomousObject only accept AUTONOMOUS_ACTION as "
                       "action value input.")
    return action

  def apply_action(self, action: Any) -> None:
    """Applies the action to the robot."""
    # Environment should not pass action other than AUTONOMOUS_ACTION.
    if action is not AUTONOMOUS_ACTION:
      raise ValueError("AutonomousObject only accept AUTONOMOUS_ACTION as "
                       "action value input.")
    position, orientation, _ = self._controller.get_action(
        self._observations_time_since_reset, self._observations)

    self._actuate_function(position, orientation)

  def receive_observation(self) -> None:
    """Updates the robot sensor readings."""
    position, orientation = (
        self._pybullet_client.getBasePositionAndOrientation(self._object_id))
    self._position = np.array(position)
    self._orientation = np.array(orientation)

  def post_control_step(self) -> None:
    """Updates internal variables. Not yet used in AutonomousObject."""
    pass

  def _reset_base_pose(self,
                       position: Union[Sequence[float], np.ndarray] = None,
                       orientation_quat: Union[Sequence[float],
                                               np.ndarray] = None):
    """Resets the base to the desired position and orientation.

    Args:
      position: The desired base position. If omitted, current location is used.
      orientation_quat: The desired base orientation in quaternion. If omitted,
        current orientation is used.
    """
    if position is None:
      position = self._position

    if orientation_quat is None:
      orientation_quat = self._orientation

    self._pybullet_client.resetBaseVelocity(self._object_id, (0, 0, 0),
                                            (0, 0, 0))
    self._pybullet_client.resetBasePositionAndOrientation(
        self._object_id, position, orientation_quat)

  def _actuate_base_pose(self, position: Union[Sequence[float], np.ndarray],
                         orientation_quat: Union[Sequence[float], np.ndarray]):
    """Actuates the base to the desired position and orientation.

    Difference of this function from _reset_base_pose() is that this function
    considers dynamics along the path and collisions along the motion path.

    Args:
      position: The desired base position.
      orientation_quat: The desired base orientation in quaternion.
    """
    self._pybullet_client.changeConstraint(
        self._constraint_id,
        position,
        jointChildFrameOrientation=orientation_quat,
        maxForce=_MAX_FORCE)

  def _reset_joint_angles(self, joint_angles=None):
    """Resets the joint angles. Not yet used in AutonomousObject."""
    del joint_angles

  @property
  def action_names(self) -> Sequence[Text]:
    """Returns a sequence of action names. Always () for AutonomousObject."""
    return ()

  @property
  def sensors(self) -> Sequence[sensor.Sensor]:
    """Returns the sensors on this robot."""
    return self._sensors

  @property
  def base_orientation_quaternion(self) -> np.ndarray:
    """Returns the base pose as a quaternion in format (x, y, z, w)."""
    return self._orientation.copy()

  @property
  def base_roll_pitch_yaw(self) -> np.ndarray:
    """Returns the base roll, pitch, and yaw angles in radians."""
    return np.array(
        self._pybullet_client.getEulerFromQuaternion(self._orientation))

  @property
  def base_position(self) -> np.ndarray:
    """Returns the base cartesian coordinates in meters."""
    return self._position.copy()

  @property
  def timestamp(self):
    """Simulation monotonic time."""
    if self._clock is None:
      raise RuntimeError("Must call set_clock() before accessing timestamp.")
    return self._clock()

  # This is need for CameraSensor.set_robot() to work.
  @property
  def pybullet_client(self):
    return self._pybullet_client

  # This is need for CameraSensor.set_robot() to work.
  @property
  def robot_id(self) -> int:
    return self._object_id
