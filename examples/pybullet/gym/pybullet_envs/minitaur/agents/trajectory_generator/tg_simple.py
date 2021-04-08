"""Trajectory Generator generates walking leg motions for a quadruped robot.

Trajectory Generator (TG) has an internal state (phase) and generates
walking-like motion for 8 motors of minitaur quadruped robot based on
parameters
such as:
 - delta time to progress the TG's internal state.
 - intensity to control amount of movement (stride length and lift of the legs).
 - waking height to control the average extension of the legs.

Each time step() is called, the internal state is progressed and 8 motor
positions are generated. This TG uses the open-loop SineController class to
provide leg positions. It is mainly a wrapper for ability to modulating the
time
and other parameters of the SineController.
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import math
import numpy as np
import gin
from pybullet_envs.minitaur.agents.trajectory_generator import controller_simple

PHASE_LOWER_BOUND = 0.0
PHASE_UPPER_BOUND = 1.0
WALK_HEIGHT_LOWER_BOUND = -0.5
WALK_HEIGHT_UPPER_BOUND = 1.0
INTENSITY_LOWER_BOUND = 0.0
INTENSITY_UPPER_BOUND = 1.5
_SWING_STANCE_LOWER_BOUND = 0.2
_SWING_STANCE_UPPER_BOUND = 5.0
_DELTA_SWING_STANCE_CAP = 0.4
_TWO_PI = math.pi * 2.0
_LEG_COUPLING_DICT = {
    "null": [],
    # All the legs are coupled.
    "all coupled": [0, 0, 0, 0],
    # Front legs and back legs are coupled separately.
    "front back": [0, 1, 0, 1],
    # Left legs and right legs are coupled separately.
    "left right": [0, 0, 1, 1],
    # Diagonal legs are coupled (i.e. trottting).
    "diagonal": [0, 1, 1, 0],
    # Each leg is indepenent.
    "decoupled": [0, 1, 2, 3]
}


@gin.configurable
class TgSimple(object):
  """TgSimple class is a simplified trajectory generator for quadruped walking.

  It returns 8 actions for quadruped slow walking behavior
  based on the parameters provided such as intensity, walking height and delta
  time. It returns its internal phase as information.
  """

  def __init__(self,
               walk_height_lower_bound=WALK_HEIGHT_LOWER_BOUND,
               walk_height_upper_bound=WALK_HEIGHT_UPPER_BOUND,
               intensity_lower_bound=INTENSITY_LOWER_BOUND,
               intensity_upper_bound=INTENSITY_UPPER_BOUND,
               swing_stance_lower_bound=_SWING_STANCE_LOWER_BOUND,
               swing_stance_upper_bound=_SWING_STANCE_UPPER_BOUND,
               integrator_coupling_mode="all coupled",
               walk_height_coupling_mode="all coupled",
               variable_swing_stance_ratio=False,
               swing_stance_ratio=1.0,
               init_leg_phase_offsets=None):
    """Initialize the trajectory generator with a controller.

    For trajectory generator, we create an asymmetric sine controller with
    parameters that was previously optimized as an open-loop controller.

    Args:
      walk_height_lower_bound: Lower bound for walking height which sets the
        default leg extension of the gait. Unit is rad, -0.5 by default.
      walk_height_upper_bound: Lower bound for walking height which sets the
        default leg extension of the gait. Unit is rad, 1.0 by default.
      intensity_lower_bound: The upper bound for intensity of the trajectory
        generator. It can be used to limit the leg movement.
      intensity_upper_bound: The upper bound for intensity of the trajectory
        generator. It can be used to limit the leg movement.
      swing_stance_lower_bound: Lower bound for the swing vs stance ratio
        parameter. Default value is 0.2.
      swing_stance_upper_bound: Upper bound for the swing vs stance ratio
        parameter. Default value is 0.2.
      integrator_coupling_mode: How the legs should be coupled for integrators.
      walk_height_coupling_mode: The same coupling mode used for walking
        heights for the legs.
      variable_swing_stance_ratio: A boolean to indicate if the swing stance
        ratio can change per time step or not.
      swing_stance_ratio: Time taken by swing phase vs stance phase. This is
        only relevant if variable_swing_stance_ratio is False.
      init_leg_phase_offsets: The initial phases of the legs. A list of 4
        variables within [0,1). The order is front-left, rear-left, front-right
        and rear-right.

    Raises:
      ValueError: If parameters are not valid values.
    """
    self._walk_height_lower_bound = walk_height_lower_bound
    self._walk_height_upper_bound = walk_height_upper_bound
    self._intensity_lower_bound = intensity_lower_bound
    self._intensity_upper_bound = intensity_upper_bound
    self._swing_stance_lower_bound = swing_stance_lower_bound
    self._swing_stance_upper_bound = swing_stance_upper_bound
    if not init_leg_phase_offsets:
      init_leg_phase_offsets = [0, 0.25, 0.5, 0.75]
    if len(init_leg_phase_offsets) != 4:
      raise ValueError("The number of leg phase offsets is not equal to 4.")
    if min(init_leg_phase_offsets) < 0 or max(init_leg_phase_offsets) >= 1:
      raise ValueError("Leg phase offsets are not within [0,1)")
    self._legs = []
    for period in init_leg_phase_offsets:
      init_phase = period * 2 * math.pi
      self._legs.append(controller_simple.SimpleLegController(init_phase))

    if integrator_coupling_mode not in _LEG_COUPLING_DICT:
      raise ValueError("Invalid integrator_coupling_mode.")
    if walk_height_coupling_mode not in _LEG_COUPLING_DICT:
      raise ValueError("Invalid walk_height_coupling_mode.")

    # Set the phase couplings and build a list of legs per phase coupling.
    self._integrator_id_per_leg = _LEG_COUPLING_DICT[integrator_coupling_mode]
    self._num_integrators = max(
        self._integrator_id_per_leg) + 1 if self._integrator_id_per_leg else 0
    self._legs_per_integrator_id = [[], [], [], []]
    for idx, phase_id in enumerate(self._integrator_id_per_leg):
      self._legs_per_integrator_id[phase_id].append(self._legs[idx])

    # For each integrator coupling, create a integrator unit.
    # For each leg controlled by that phase generator, mark the phase offset.
    self._integrator_units = []
    for legs_per_integrator in self._legs_per_integrator_id:
      if legs_per_integrator:
        circular_integrator = CircularAsymmetricalIntegratorUnit(
            legs_per_integrator[0].phase)
        self._integrator_units.append(circular_integrator)
        for leg in legs_per_integrator:
          leg.phase_offset = leg.phase - circular_integrator.phase

    # Set the walking heights couplings.
    self._walk_height_id_per_leg = _LEG_COUPLING_DICT[walk_height_coupling_mode]
    self._num_walk_heights = max(
        self._walk_height_id_per_leg) + 1 if self._walk_height_id_per_leg else 0
    self._variable_swing_stance_ratio = variable_swing_stance_ratio
    self._swing_stance_ratio = swing_stance_ratio

  def reset(self):
    """Resets leg phase offsets to their initial values."""
    for leg in self._legs:
      leg.reset()
    for circular_integrator in self._integrator_units:
      circular_integrator.reset()

  def get_parameter_bounds(self):
    """Lower and upper bounds for the parameters generator's parameters.

    Returns:
      2-tuple of:
      - Lower bounds for the parameters such as intensity, walking height and
      lift fraction.
      - Upper bounds for the same parameters.
    """
    lower_bounds = [self._intensity_lower_bound]
    upper_bounds = [self._intensity_upper_bound]
    lower_bounds += [self._walk_height_lower_bound] * self._num_walk_heights
    upper_bounds += [self._walk_height_upper_bound] * self._num_walk_heights
    lower_bounds += [self._swing_stance_lower_bound
                    ] * self._variable_swing_stance_ratio
    upper_bounds += [self._swing_stance_upper_bound
                    ] * self._variable_swing_stance_ratio

    return lower_bounds, upper_bounds

  def get_actions(self, delta_real_time, tg_params):
    """Get actions for 8 motors after increasing the phase delta_time.

    Args:
      delta_real_time: Time in seconds that have actually passed since the last
        step of the trajectory generator.
      tg_params: An ndarray of the parameters for generating the trajectory. The
        parameters must be in the correct order (time_scale, intensity,
        walking_height, and swing vs stance)

    Raises:
      ValueError: In case the input dimension does not match expected.
    Returns:
      The rotations for all the 8 motors for this time step
      returned in an array [front_left_motor_1, front_left_motor_2, etc].
    """
    speeds, intensity, heights, swing_stance_ratio = self._process_tg_params(
        tg_params)
    # Adjust the swing stance ratio of the controller (used for all four legs).
    if swing_stance_ratio:
      self.adjust_swing_stance_ratio(swing_stance_ratio)
    # Adjust the walking height, intensity and swing vs stance of the legs.
    for idx, leg in enumerate(self._legs):
      leg.adjust_intensity(intensity)
      if heights:
        leg.adjust_center_extension(heights[self._walk_height_id_per_leg[idx]])

    # Progress all the phase generators based on delta time.
    for idx, integrator_unit in enumerate(self._integrator_units):
      integrator_unit.progress_phase(speeds[idx] * delta_real_time,
                                     self._swing_stance_ratio)

    # Set the phases for the legs based on their offsets with phase generators.
    for phase_id, leg_list in enumerate(self._legs_per_integrator_id):
      for leg in leg_list:
        delta_period = leg.phase_offset / (2.0 * math.pi)
        leg.phase = self._integrator_units[phase_id].calculate_progressed_phase(
            delta_period, self._swing_stance_ratio)

    # Calculate swingextend and convert it to the motor rotations.
    actions = []
    for idx, leg in enumerate(self._legs):
      swing, extend = leg.get_swing_extend()
      actions.extend([swing, extend])
    return actions

  def _process_tg_params(self, tg_params):
    """Process the trajectory generator parameters and split them.

    Args:
      tg_params: A list consisting of time_scales, intensity, walking_heights,
        swing_stance_ratio. The size depends on the configuration and inital
        flags.

    Returns:
      time_scales: A list of multipliers of delta time (one per integrator).
      intensity: Intensity of the trajectory generator (one variable).
      walking_heights: Walking heights used for the legs. The length depends on
        the coupling between the legs selected at the initialization.
      swing_stance_ratio: The ratio of the speed of the leg during swing stance
      vs stance phase.
    """

    # Check if the given input's dimension matches the expectation considering
    # the number of parameters the trajectory generator uses.
    if isinstance(tg_params, np.ndarray):
      tg_params = tg_params.tolist()
    expected_action_dim = 1 + self._num_integrators + self._num_walk_heights
    if self._variable_swing_stance_ratio:
      expected_action_dim += 1
    if len(tg_params) != expected_action_dim:
      raise ValueError(
          "Action dimension does not match the expectation {} vs {}".format(
              len(tg_params), expected_action_dim))
    # Split input into different parts based on type. The order must match the
    # order given by the order in get_parameter_bounds
    time_scales = tg_params[0:self._num_integrators]
    intensity = tg_params[self._num_integrators]
    walking_heights = tg_params[(self._num_integrators + 1):(
        1 + self._num_integrators + self._num_walk_heights)]
    swing_stance_ratio = None
    if self._variable_swing_stance_ratio:
      swing_stance_ratio = tg_params[1 + self._num_integrators +
                                     self._num_walk_heights]

    return time_scales, intensity, walking_heights, swing_stance_ratio

  def get_state(self):
    """Returns a list of floats representing the phase of the controller.

    The phase of the controller is composed of the phases of the integrators.
    For each integrator, the phase is composed of 2 floats that represents the
    sine and cosine of the phase of that integrator.

    Returns:
      List containing sine and cosine of the phases of all the integrators.
    """
    return [x for y in self._integrator_units for x in y.get_state()]

  def get_state_lower_bounds(self):
    """Lower bounds for the internal state.

    Returns:
      The list containing the lower bounds.
    """
    return [PHASE_LOWER_BOUND] * 2 * self._num_integrators

  def get_state_upper_bounds(self):
    """Upper bounds for the internal state.

    Returns:
      The list containing the upper bounds.
    """
    return [PHASE_UPPER_BOUND] * 2 * self._num_integrators

  def adjust_swing_stance_ratio(self, target_swing_stance_ratio):
    """Adjust the parameter swing_stance_ratio towards a given target value.

    Args:
      target_swing_stance_ratio: The target value for the ratio between swing
        and stance phases.
    """
    delta = min(_DELTA_SWING_STANCE_CAP,
                target_swing_stance_ratio - self._swing_stance_ratio)
    self._swing_stance_ratio += delta

  @property
  def num_integrators(self):
    """Gets the number of integrators used based on coupling mode."""
    return self._num_integrators


class CircularAsymmetricalIntegratorUnit(object):
  """A circular integrator with asymmetry between first and second half.

  An integrator is a memory unit that accumulates the given parameter at every
  time step.
  A circular integrator is when the integrator cycles within [0,2pi].
  The phase of a circular integrator indicates the accumulated number and it is
  stored as fmod of 2Pi.
  Asymmetrical circular integrator has a further characteristic where it
  distinguishes between the first half of the period vs the second half. It
  allows the integrator to move at different speeds during these two periods.
  From a locomotion perspective these two halves of the period can be considered
  as swing and stance phases. The speed difference is calculated using the
  variable swing_stance_ratio provided at every time step.
  CircularAsymmetricalIntegratorUnit can be used to control one or multiple legs
  depending on the preference. If more than one leg is assigned to a single unit
  the other legs are calculated based on their initial phase difference.
  """

  def __init__(self, init_phase=0):
    self._init_phase = init_phase
    self.reset()

  def reset(self):
    self.phase = self._init_phase

  def calculate_progressed_phase(self, delta_period, swing_stance_speed_ratio):
    """Calculate a hypotethical phase based on the current phase and args.

    This is used to both calculate the new phase, as well as the current phase
    of the other legs with a given offset of delta_period.

    Args:
      delta_period: The fraction of the period to add to the current phase of
        the integrator. If set to 1, the integrator will accomplish one full
        period and return the same phase. The calculated phase will depend on
        the current phase (if it is in first half vs second half) and swing vs
        stance speed ratio.
      swing_stance_speed_ratio: The ratio of the speed of the phase when it is
        in swing (second half) vs stance (first half). Set to 1.0 by default,
        making it symettric, same as a classical integrator.

    Returns:
      The new phase between 0 and 2 * pi.
    """
    stance_speed_coef = (
        swing_stance_speed_ratio + 1) * 0.5 / swing_stance_speed_ratio
    swing_speed_coef = (swing_stance_speed_ratio + 1) * 0.5
    delta_left = delta_period
    new_phase = self.phase
    while delta_left > 0:
      if 0 <= new_phase < math.pi:
        delta_phase_multiplier = stance_speed_coef * _TWO_PI
        new_phase += delta_left * delta_phase_multiplier
        delta_left = 0
        if new_phase < math.pi:
          delta_left = 0
        else:
          delta_left = (new_phase - math.pi) / delta_phase_multiplier
          new_phase = math.pi
      else:
        delta_phase_multiplier = swing_speed_coef * _TWO_PI
        new_phase += delta_left * delta_phase_multiplier
        if math.pi <= new_phase < _TWO_PI:
          delta_left = 0
        else:
          delta_left = (new_phase - _TWO_PI) / delta_phase_multiplier
          new_phase = 0
    return math.fmod(new_phase, _TWO_PI)

  def progress_phase(self, delta_period, swing_stance_ratio):
    """Updates the phase based on the current phase, delta period and ratio.

    Args:
      delta_period: The fraction of the period to add to the current phase of
        the integrator. If set to 1, the integrator will accomplish one full
        period and return the same phase. The calculated phase will depend on
        the current phase (if it is in first half vs second half) and swing vs
        stance speed ratio.
      swing_stance_ratio: The ratio of the speed of the phase when it is in
        swing (second half) vs stance (first half). Set to 1.0 by default,
        making it symettric, same as a classical integrator.
    """
    self.phase = self.calculate_progressed_phase(delta_period,
                                                 swing_stance_ratio)

  def get_state(self):
    """Returns the sin and cos of the phase as state."""
    return [(math.cos(self.phase) + 1) / 2.0, (math.sin(self.phase) + 1) / 2.0]
