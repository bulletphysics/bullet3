# Lint as: python3
"""The configuration parameters for our robots."""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import enum
from typing import Sequence, Union
import dataclasses
import gin
import numpy as np


@gin.constants_from_enum
class MotorControlMode(enum.Enum):
  """The supported motor control modes."""
  POSITION = 1,

  # Apply motor torques directly.
  TORQUE = 2,

  # Apply a tuple (q, qdot, kp, kd, tau) for each motor. Here q, qdot are motor
  # position and velocities. kp and kd are PD gains. tau is the additional
  # motor torque. This is the most flexible control mode.
  HYBRID = 3,

  # PWM mode is only availalbe for Minitaur
  PWM = 4


# TODO(b/127675924): Group other parameters in the named attrib class.

# Each hybrid action is a tuple (position, position_gain, velocity,
# velocity_gain, torque)
HYBRID_ACTION_DIMENSION = 5


class HybridActionIndex(enum.Enum):
  # The index of each component within the hybrid action tuple.
  POSITION = 0
  POSITION_GAIN = 1
  VELOCITY = 2
  VELOCITY_GAIN = 3
  TORQUE = 4


@gin.configurable
class MotorLimits(object):
  """The data class for motor limits."""

  def __init__(
      self,
      angle_lower_limits: Union[float, Sequence[float]] = float('-inf'),
      angle_upper_limits: Union[float, Sequence[float]] = float('inf'),
      velocity_lower_limits: Union[float, Sequence[float]] = float('-inf'),
      velocity_upper_limits: Union[float, Sequence[float]] = float('inf'),
      torque_lower_limits: Union[float, Sequence[float]] = float('-inf'),
      torque_upper_limits: Union[float, Sequence[float]] = float('inf'),
  ):
    """Initializes the class."""
    self.angle_lower_limits = angle_lower_limits
    self.angle_upper_limits = angle_upper_limits
    self.velocity_lower_limits = velocity_lower_limits
    self.velocity_upper_limits = velocity_upper_limits
    self.torque_lower_limits = torque_lower_limits
    self.torque_upper_limits = torque_upper_limits


@gin.constants_from_enum
class WheeledRobotControlMode(enum.Enum):
  """The control mode for wheeled robots."""
  # Controls the base of the robot (i.e. in kinematic mode.) or the base wheels
  # using motor commands.
  BASE = 1
  # Controls arm only
  ARM = 2
  # Controls both base and arm
  BASE_AND_ARM = 3
  # Controls both base and head
  BASE_AND_HEAD = 4
  # Controls the non-wheel motors. This include arms and heads.
  BODY = 5
  # Controls all degrees of freedom, i.e. the base and arm/head simultaneously.
  ALL = 6
  # High-level navigation target.
  NAVIGATION_TARGET = 7
  # Individually addressable actions for body joints, with nested dict actions.
  ADDRESSABLE = 8


@dataclasses.dataclass
class TwistActionLimits:
  """The data class for twist action limits.

  Common abbreviations used in variable names suffix:
    mps = Meters per Second
    rps = Radians per Second
  """
  max_linear_mps: float
  min_linear_mps: float
  max_angular_rps: float
  min_angular_rps: float



