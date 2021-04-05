"""Asymmetric sine controller for quadruped locomotion.

Asymmetric sine uses cosine and sine waves to generate swinging and extension
for leg motion. It's asymmetric because sine waves are split into two phases
(swing forward and stance) and these phases have different frequencies according
to what proportion of a period will be spend on swinging forward forward
swinging backwards. In addition, the sine wave for extension has different
amplitudes during these two phases.
"""
import math

TWO_PI = 2 * math.pi
_DEFAULT_LEG_AMPLITUDE_EXTENSION = -0.02
_DEFAULT_LEG_AMPLITUDE_SWING = 0.5
_DEFAULT_LEG_AMPLITUDE_LIFT = 0.9
_DEFAULT_WALKING_HEIGHT = 0.0
_DEFAULT_LEG_CENTER_SWING = 0.0
_DELTA_CENTER_EXTENSION_CAP = 1
_DELTA_INTENSITY_CAP = 0.1


class SimpleLegController(object):
  """Controller that gives swing and extension based on phase and parameters.


  The controller returns the swing-extend pair based on a parameterized
  ellipsoid trajectory that depends on center of motion, amplitude and phase.
  The parameters are
    amplitude_extension: Amplitude for extension during stance (phase < pi).
    amplitude_lift: Amplitude for extension during swing (phase > pi).
    amplitude_swing: Amplitude for swing.
    center_extension: The value extension signal oscillates around.
    center_swing: The value swing signal oscillates around.
    intensity: A coefficient that scales the motion of the legs.
  The formula to calculate motion and more detailed information about these
  parameters can be found at go/pmtg-refactored.
  """

  def __init__(self, init_phase=0):
    self.amplitude_extension = _DEFAULT_LEG_AMPLITUDE_EXTENSION
    self.amplitude_swing = _DEFAULT_LEG_AMPLITUDE_SWING
    self.amplitude_lift = _DEFAULT_LEG_AMPLITUDE_LIFT
    self.center_extension = _DEFAULT_WALKING_HEIGHT
    self.center_swing = _DEFAULT_LEG_CENTER_SWING
    self.intensity = 1.0
    self._init_phase = init_phase
    self.phase = init_phase
    self.phase_offset = 0

  def reset(self):
    self.phase = self._init_phase

  def get_swing_extend(self):
    """Returns the swing and extend parameters for the leg.

    Returns:
      swing: Desired swing of the leg.
      extend: Desired extension amount of the leg.
    """

    # Increase default extension by the extra extension scaled by intensity.
    # Extend reduces to default center extension when intensity goes to 0,
    # because we prefer the legs to stay at walking height when intensity is
    # 0.
    amplitude_extension = self.amplitude_extension
    # The leg is in swing phase when phase > pi.
    if self.phase > math.pi:
      amplitude_extension = self.amplitude_lift
    extend = self.center_extension + (
        amplitude_extension * math.sin(self.phase)) * self.intensity
    # Calculate the swing based on the signal and scale it with intensity.
    # Swing reduces to 0 when intensity goes to 0, because we would prefer the
    # legs to stay neutral (standing position instead of center swing) when
    # intensity is 0.
    swing = self.center_swing + self.amplitude_swing * math.cos(self.phase)
    swing *= self.intensity

    return swing, extend

  def adjust_center_extension(self, target_center_extension):
    delta = min(_DELTA_CENTER_EXTENSION_CAP,
                target_center_extension - self.center_extension)
    self.center_extension += delta

  def adjust_intensity(self, target_intensity):
    delta = min(_DELTA_INTENSITY_CAP, target_intensity - self.intensity)
    self.intensity += delta
