"""Gait pattern planning module."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import logging
import math
from typing import Any, Sequence

import gin
import numpy as np

from pybullet_envs.minitaur.agents.baseline_controller import gait_generator

_DEFAULT_INITIAL_LEG_STATE = (
    gait_generator.LegState.STANCE,
    gait_generator.LegState.STANCE,
    gait_generator.LegState.STANCE,
    gait_generator.LegState.STANCE,
)

_NOMINAL_STANCE_DURATION = (0.25, 0.25, 0.25, 0.25)
_NOMINAL_DUTY_FACTOR = (0.6, 0.6, 0.6, 0.6)
_TROTTING_LEG_PHASE = (0, 0.5, 0.5, 0)
_NOMINAL_CONTACT_DETECTION_PHASE = 0.4


@gin.configurable
class OpenloopGaitGenerator(gait_generator.GaitGenerator):
  """Generates openloop gaits for quadruped robots.

  A flexible open-loop gait generator. Each leg has its own cycle and duty
  factor. And the state of each leg alternates between stance and swing. One can
  easily formuate a set of common quadruped gaits like trotting, pacing,
  pronking, bounding, etc by tweaking the input parameters.
  """

  def __init__(
      self,
      robot: Any,
      stance_duration: Sequence[float] = _NOMINAL_STANCE_DURATION,
      duty_factor: Sequence[float] = _NOMINAL_DUTY_FACTOR,
      initial_leg_phase: Sequence[float] = _TROTTING_LEG_PHASE,
      contact_detection_force_threshold: float = 0,
      contact_detection_phase_threshold:
      float = _NOMINAL_CONTACT_DETECTION_PHASE,
  ):
    """Initializes the class.

    Args:
      robot: A quadruped robot that at least implements the GetFootContacts API
        and num_legs property.
      stance_duration: The desired stance duration.
      duty_factor: The ratio  stance_duration / total_gait_cycle.
      initial_leg_phase: The desired initial phase [0, 1] of the legs within the
        full swing + stance cycle.
      contact_detection_force_threshold: The minimal contact force required to
        detect if a foot is in contact with the ground. For real robots this
        needs to be larger (i.e. 25 for Laikago).
      contact_detection_phase_threshold: Updates the state of each leg based on
        contact info, when the current normalized phase is greater than this
        threshold. This is essential to remove false positives in contact
        detection when phase switches. For example, a swing foot at at the
        beginning of the gait cycle might be still on the ground.
    """
    self._robot = robot
    self._stance_duration = stance_duration
    self._duty_factor = duty_factor
    self._swing_duration = np.array(stance_duration) / np.array(
        duty_factor) - np.array(stance_duration)
    if len(initial_leg_phase) != len(
        list(self._robot.urdf_loader.get_end_effector_id_dict().values())):
      raise ValueError(
          "The number of leg phases should be the same as number of legs.")
    self._initial_leg_phase = initial_leg_phase

    self._initial_leg_state = _DEFAULT_INITIAL_LEG_STATE
    self._next_leg_state = []
    # The ratio in cycle is duty factor if initial state of the leg is STANCE,
    # and 1 - duty_factory if the initial state of the leg is SWING.
    self._initial_state_ratio_in_cycle = []
    for state, duty in zip(self._initial_leg_state, duty_factor):
      assert state == gait_generator.LegState.STANCE
      self._initial_state_ratio_in_cycle.append(duty)
      self._next_leg_state.append(gait_generator.LegState.SWING)

    self._contact_detection_force_threshold = contact_detection_force_threshold
    self._contact_detection_phase_threshold = contact_detection_phase_threshold

    # The normalized phase within swing or stance duration.
    self._normalized_phase = None

    # The current leg state, when contact is considered.
    self._leg_state = None

    # The desired leg state (i.e. SWING or STANCE).
    self._desired_leg_state = None

    self.reset(0)

  def reset(self, current_time):
    # The normalized phase within swing or stance duration.
    self._normalized_phase = np.zeros(
        len(list(self._robot.urdf_loader.get_end_effector_id_dict().values())))
    self._leg_state = list(self._initial_leg_state)
    self._desired_leg_state = list(self._initial_leg_state)

  @property
  def desired_leg_state(self) -> Sequence[gait_generator.LegState]:
    """The desired leg SWING/STANCE states.

    Returns:
      The SWING/STANCE states for all legs.

    """
    return self._desired_leg_state

  @property
  def leg_state(self) -> Sequence[gait_generator.LegState]:
    """The leg state after considering contact with ground.

    Returns:
      The actual state of each leg after accounting for contacts.
    """
    return self._leg_state

  @property
  def swing_duration(self) -> Sequence[float]:
    return self._swing_duration

  @property
  def stance_duration(self) -> Sequence[float]:
    return self._stance_duration

  @property
  def normalized_phase(self) -> Sequence[float]:
    """The phase within the current swing or stance cycle.

    Reflects the leg's phase within the curren swing or stance stage. For
    example, at the end of the current swing duration, the phase will
    be set to 1 for all swing legs. Same for stance legs.

    Returns:
      Normalized leg phase for all legs.

    """
    return self._normalized_phase

  def update(self, current_time):
    contact_state = [
        np.linalg.norm(contact_force) > self._contact_detection_force_threshold
        for contact_force in self._robot.feet_contact_forces()
    ]

    for leg_id in range(
        len(list(self._robot.urdf_loader.get_end_effector_id_dict().values()))):
      # Here is the explanation behind this logic: We use the phase within the
      # full swing/stance cycle to determine if a swing/stance switch occurs
      # for a leg. The threshold value is the "initial_state_ratio_in_cycle" as
      # explained before. If the current phase is less than the initial state
      # ratio, the leg is either in the initial state or has switched back after
      # one or more full cycles.
      full_cycle_period = (
          self._stance_duration[leg_id] / self._duty_factor[leg_id])
      # To account for the non-zero initial phase, we offset the time duration
      # with the effect time contribution from the initial leg phase.
      augmented_time = current_time + self._initial_leg_phase[
          leg_id] * full_cycle_period
      phase_in_full_cycle = math.fmod(augmented_time,
                                      full_cycle_period) / full_cycle_period
      ratio = self._initial_state_ratio_in_cycle[leg_id]
      if phase_in_full_cycle < ratio:
        self._desired_leg_state[leg_id] = self._initial_leg_state[leg_id]
        self._normalized_phase[leg_id] = phase_in_full_cycle / ratio
      else:
        # A phase switch happens for this leg.
        self._desired_leg_state[leg_id] = self._next_leg_state[leg_id]
        self._normalized_phase[leg_id] = (phase_in_full_cycle - ratio) / (1 -
                                                                          ratio)

      self._leg_state[leg_id] = self._desired_leg_state[leg_id]

      # No contact detection at the beginning of each SWING/STANCE phase.
      if (self._normalized_phase[leg_id] <
          self._contact_detection_phase_threshold):
        continue
      if (self._leg_state[leg_id] == gait_generator.LegState.SWING and
          contact_state[leg_id]):
        logging.info("early touch down detected")
        self._leg_state[leg_id] = gait_generator.LegState.EARLY_CONTACT
      if (self._leg_state[leg_id] == gait_generator.LegState.STANCE and
          not contact_state[leg_id]):
        self._leg_state[leg_id] = gait_generator.LegState.LOSE_CONTACT
