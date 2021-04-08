# Lint as: python3
"""The abstract robot class."""

import abc
from typing import Optional, Sequence

# Action names for robots operating kinematically.
LINEAR_VELOCITY = "linear_velocity"
ANGULAR_VELOCITY = "angular_velocity"


class RobotBase(metaclass=abc.ABCMeta):
  """The base class for all robots used in the mobility team."""

  @abc.abstractmethod
  def reset(
      self,
      base_position: Optional[Sequence[float]] = None,
      base_orientation_quaternion: Optional[Sequence[float]] = None) -> None:
    """Resets the states (e.g. pose and sensor readings) of the robot.

    This is called at the start of each episode by the environment.

    Args:
      base_position: Robot base position after reset. If None, robot stay where
        it was after reset. For robot that does not support reset with position
        change, a ValueError should be raised.
      base_orientation_quaternion: Robot base orientation after reset. If None,
        robot stays in pre-reset orientation. For robot that does not support
        reset with orientation change, a ValueError should be raised.
    """
    pass

  @abc.abstractmethod
  def terminate(self):
    """Shuts down the robot."""
    pass

  @abc.abstractmethod
  def pre_control_step(self, action):
    """Processes the input action before the action repeat loop.

    We assume that an action sent to the real robot is sticky, i.e. it will be
    executed until a new action is received after some time. To simulate this,
    we introduced the action_repeat parameter, to reflect how many time steps it
    takes for the policy to generate a new action. That is, for each control
    step, the simulation contains an inner loop:

      robot.pre_control_step(action)  # Smooth or interpolate the action
      for i in range(action_repeat):
        robot.apply_action(action)
        bullet.stepSimulation(time_step)  # Step the sim for one time step
        robot.receive_observation()  # Update the sensor observations
      robot.post_control_step() # Update some internal variables.

    Args:
      action: Data type depends on the robot. Can be desired motor
        position/torques for legged robots, or desired velocity/angular velocity
        for wheeled robots.
    """
    pass

  @abc.abstractmethod
  def apply_action(self, action):
    """Applies the action to the robot."""
    pass

  @abc.abstractmethod
  def receive_observation(self):
    """Updates the robot sensor readings."""
    pass

  @abc.abstractmethod
  def post_control_step(self):
    """Updates some internal variables such as step counters."""
    pass

  @property
  def action_space(self):
    """The action spec of the robot."""
    raise NotImplementedError("action_space is not implemented")

  @property
  @abc.abstractmethod
  def action_names(self):
    """Name of each action in the action_space.

    This is a structure of strings with the same shape as the action space,
    where each string describes the corresponding element of the action space
    (for example, a kinematic robot might return ("linear_velocity",
    "angular_velocity")). Used for logging in the safety layer.
    """

  @property
  def sensors(self):
    """Returns the sensors on this robot.

    Sensors are the main interface between the robot class and the gym
    environment. Sensors can return what the robot can measure (e.g.
    joint angles, IMU readings), and can represent more general quantities, i.e.
    the last action taken, that can be part of the observation space.
    Sensor classes are used by the robot class to the specify its observation
    space.

    """
    raise NotImplementedError("sensors property not implemented")

  @property
  def base_orientation_quaternion(self):
    """Returns the base pose as a quaternion in format (x, y, z, w).

    These properties differ from the sensor interfaces, as they represent
    the built-in measurable quantities. We assume most robots have an IMU at
    its base to measure the base pose. Actually, some sensor classes like the
    base pose sensor and joint angle sensor will call these built-in methods. In
    general, how these quantities can be extracted depends on the specific real
    robots.

    """
    raise NotImplementedError("base_orientation_quaternion is not implemented")

  @property
  def base_roll_pitch_yaw(self):
    """Returns the base roll, pitch, and yaw angles."""
    raise NotImplementedError("base_roll_pitch_yaw is not implemented")

  @property
  def base_roll_pitch_yaw_rate(self):
    raise NotImplementedError("base_roll_pitch_yaw_rate is not implemented")

  @property
  def base_position(self):
    raise NotImplementedError("base_position is not implemented")
