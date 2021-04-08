"""Trajectory Generator for in-place stepping motion for quadruped robot."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import math
import numpy as np

TWO_PI = 2 * math.pi


def _get_actions_asymmetric_sine(phase, tg_params):
  """Returns the leg extension given current phase of TG and parameters.

  Args:
    phase: a number in [0, 2pi) representing current leg phase
    tg_params: a dictionary of tg parameters:
      stance_lift_cutoff -- switches the TG between stance (phase < cutoff) and
                            lift (phase > cutoff) phase
      amplitude_swing -- amplitude in swing phase
      amplitude_lift -- amplitude in lift phase
      center_extension -- center of leg extension
  """
  stance_lift_cutoff = tg_params['stance_lift_cutoff']
  a_prime = np.where(phase < stance_lift_cutoff, tg_params['amplitude_stance'],
                     tg_params['amplitude_lift'])
  scaled_phase = np.where(
      phase > stance_lift_cutoff, np.pi + (phase - stance_lift_cutoff) /
      (TWO_PI - stance_lift_cutoff) * np.pi, phase / stance_lift_cutoff * np.pi)
  return tg_params['center_extension'] + a_prime * np.sin(scaled_phase)


def step(current_phases, leg_frequencies, dt, tg_params):
  """Steps forward the in-place trajectory generator.

  Args:
    current_phases: phases of each leg.
    leg_frequencies: the frequency to proceed the phase of each leg.
    dt: amount of time (sec) between consecutive time steps.
    tg_params: a set of parameters for trajectory generator, see the docstring
      of "_get_actions_asymmetric_sine" for details.

  Returns:
    actions: leg swing/extensions as output by the trajectory generator.
    new_state: new swing/extension.
  """
  new_phases = np.fmod(current_phases + TWO_PI * leg_frequencies * dt, TWO_PI)
  extensions = []
  for leg_id in range(4):
    extensions.append(
        _get_actions_asymmetric_sine(new_phases[..., leg_id], tg_params))
  return new_phases, extensions


def reset():
  return np.array([0, np.pi * 0.5, np.pi, np.pi * 1.5])
