"""A dummy gait generator module for storing gait patterns from higher-level controller."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import copy
from typing import Any, Sequence

import gin

from pybullet_envs.minitaur.agents.baseline_controller import gait_generator

LAIKAGO_STANDING = (
    gait_generator.LegState.STANCE,
    gait_generator.LegState.STANCE,
    gait_generator.LegState.STANCE,
    gait_generator.LegState.STANCE,
)


@gin.configurable
class DummyGaitGenerator(gait_generator.GaitGenerator):
  """A module for storing quadruped gait patterns from high-level controller.

  This module stores the state for each leg of a quadruped robot. The data is
  used by the stance leg controller to determine the appropriate contact forces.
  A high-level controller, such as a neural network policy, can be used to
  control the gait pattern and set corresponding leg states.
  """

  def __init__(
      self,
      robot: Any,
      initial_leg_state: Sequence[gait_generator.LegState] = LAIKAGO_STANDING,
  ):
    """Initializes the class.

    Args:
      robot: A quadruped robot.
      initial_leg_state: The desired initial swing/stance state of legs indexed
        by their id.
    """
    self._robot = robot
    if len(initial_leg_state) != len(
        list(self._robot.urdf_loader.get_end_effector_id_dict().values())):
      raise ValueError(
          "The number of leg states should be the same of number of legs.")
    self._initial_leg_state = initial_leg_state
    self._leg_state = list(initial_leg_state)
    self._desired_leg_state = list(initial_leg_state)

    self.reset(0)

  def reset(self, current_time):
    del current_time
    self._leg_state = list(self._initial_leg_state)
    self._desired_leg_state = list(self._initial_leg_state)

  @property
  def desired_leg_state(self) -> Sequence[gait_generator.LegState]:
    """The desired leg SWING/STANCE states.

    Returns:
      The SWING/STANCE states for all legs.

    """
    return self._desired_leg_state

  @desired_leg_state.setter
  def desired_leg_state(self, state):
    self._desired_leg_state = copy.deepcopy(state)

  @property
  def leg_state(self) -> Sequence[gait_generator.LegState]:
    """The leg state after considering contact with ground.

    Returns:
      The actual state of each leg after accounting for contacts.
    """
    return self._leg_state

  @leg_state.setter
  def leg_state(self, state):
    self._leg_state = copy.deepcopy(state)

