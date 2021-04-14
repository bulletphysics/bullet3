"""Definitions of safety layer data types."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import attr


@attr.s
class Bound(object):
  """Struct for inclusive lower and upper bounds."""
  lower = attr.ib(type=float, default=0)
  upper = attr.ib(type=float, default=0)

  @upper.validator  # pytype: disable=attribute-error
  def _upper_greator_equal_to_lower(self, attribute, value):
    del attribute
    assert value >= self.lower, (
        "upper bound {} is less than lower bound {}".format(value, self.lower))


@attr.s
class SafetyConfig(object):
  """Struct to configure the safety module."""
  motor_position_bound = attr.ib(type=list)
  motor_position_gain_bound = attr.ib(type=list)
  motor_velocity_bound = attr.ib(type=list)
  motor_velocity_gain_bound = attr.ib(type=list)
  motor_torque_bound = attr.ib(type=list)
  timestamp_delta_bound = attr.ib(type=Bound)
  motor_average_abs_velocity_bound = attr.ib(type=list)
  motor_average_abs_power_bound = attr.ib(type=list)
  state_action_timestamp_delta_bound = attr.ib(type=float)
  motor_delta_position_bound = attr.ib(type=list)
  motor_average_abs_delta_position_bound = attr.ib(type=list)


@attr.s
class MotorState(object):
  """A generic type for motor state.

  Motor states are what we can potentially read from the motor encoder or
  firmware APIs.

  """
  timestamp = attr.ib(type=float, default=None)
  position = attr.ib(type=float, default=None)
  position_gain = attr.ib(type=float, default=None)
  velocity = attr.ib(type=float, default=None)
  velocity_gain = attr.ib(type=float, default=None)
  torque = attr.ib(type=float, default=None)


@attr.s
class MotorAction(object):
  """A generic type for motor action.

   Motor actions are the potential command structure we can send to the motor.
   While similar to MotorState, they are logically very different entities.
  """
  timestamp = attr.ib(type=float, default=None)
  position = attr.ib(type=float, default=None)
  position_gain = attr.ib(type=float, default=None)
  velocity = attr.ib(type=float, default=None)
  velocity_gain = attr.ib(type=float, default=None)
  torque = attr.ib(type=float, default=None)
