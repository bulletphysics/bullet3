"""Gait pattern planning module."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import abc
import enum


class LegState(enum.Enum):
  """The state of a leg during locomotion."""
  SWING = 0
  STANCE = 1
  # A swing leg that collides with the ground.
  EARLY_CONTACT = 2
  # A stance leg that loses contact.
  LOSE_CONTACT = 3


class GaitGenerator(object):  # pytype: disable=ignored-metaclass
  """Generates the leg swing/stance pattern for the robot."""

  __metaclass__ = abc.ABCMeta

  @abc.abstractmethod
  def reset(self, current_time):
    pass

  @abc.abstractmethod
  def update(self, current_time):
    pass
