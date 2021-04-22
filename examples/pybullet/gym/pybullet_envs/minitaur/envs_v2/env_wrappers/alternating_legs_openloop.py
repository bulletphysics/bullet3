"""A trajectory generator that return signals for alternating legs."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import math
import attr
import gin
from gym import spaces
import numpy as np
from pybullet_envs.minitaur.envs.utilities import laikago_pose_utils

TROT_GAIT = "trot"
PACE_GAIT = "pace"
NUM_MOTORS_LAIKAGO = 12
STD_FOR_GAUSSIAN_TRAJECTORY = 0.15
MOTION_FREQUENCY = 1.0
MOTION_AMPLITUDE = 0.25
ACTION_BOUND = 0.25


# TODO(b/131193449): Add a test to this class.
@gin.configurable
class LaikagoAlternatingLegsTrajectoryGenerator(object):
  """A trajectory generator that return signals for alternating legs."""

  def __init__(
      self,
      init_abduction=laikago_pose_utils.LAIKAGO_DEFAULT_ABDUCTION_ANGLE,
      init_hip=laikago_pose_utils.LAIKAGO_DEFAULT_HIP_ANGLE,
      init_knee=laikago_pose_utils.LAIKAGO_DEFAULT_KNEE_ANGLE,
      amplitude=MOTION_AMPLITUDE,
      frequency=MOTION_FREQUENCY,
      gait=PACE_GAIT,  # can be TROT_GAIT or PACE_GAIT
  ):
    """Initializes the controller."""
    self._pose = np.array(
        attr.astuple(
            laikago_pose_utils.LaikagoPose(
                abduction_angle_0=init_abduction,
                hip_angle_0=init_hip,
                knee_angle_0=init_knee,
                abduction_angle_1=init_abduction,
                hip_angle_1=init_hip,
                knee_angle_1=init_knee,
                abduction_angle_2=init_abduction,
                hip_angle_2=init_hip,
                knee_angle_2=init_knee,
                abduction_angle_3=init_abduction,
                hip_angle_3=init_hip,
                knee_angle_3=init_knee)))
    action_high = np.array([ACTION_BOUND] * NUM_MOTORS_LAIKAGO)
    self.action_space = spaces.Box(-action_high, action_high, dtype=np.float32)
    self.amplitude = amplitude
    self.period = 1.0 / frequency
    self.gait = gait

  def _alternating_legs_trajectory(self, t):
    """The reference trajectory of each joint when alternating legs.

    Args:
      t: The time since the latest robot reset.

    Returns:
      An array of 12 desired motor angles.
    """
    phase_in_period = (t % self.period) / self.period
    is_first_half_gait = phase_in_period < 0.5
    if self.gait == TROT_GAIT and is_first_half_gait:
      phases = [0, 1, 1, 0]  # 0 means stance and 1 means retraction.
    elif self.gait == TROT_GAIT and not is_first_half_gait:
      phases = [1, 0, 0, 1]
    elif self.gait == PACE_GAIT and is_first_half_gait:
      phases = [0, 1, 0, 1]
    elif self.gait == PACE_GAIT and not is_first_half_gait:
      phases = [1, 0, 1, 0]
    else:
      raise ValueError("{} gait is not supported in alternating legs.".format(
          self.gait))

    phase_step_center = 0.25 if is_first_half_gait else 0.75
    std = STD_FOR_GAUSSIAN_TRAJECTORY
    # Uses Gaussian instead of sine for gentle foot touch down.
    # The following joint angles are added to self._pose.
    retract_hip_angle = self.amplitude * math.exp(
        -(phase_in_period - phase_step_center) *
        (phase_in_period - phase_step_center) / (std * std))
    retract_knee_angle = -2.0 * retract_hip_angle
    retract_abduction_angle = 0.0
    stance_hip_angle = 0.0
    stance_knee_angle = 0.0
    stance_abduction_angle = 0.0
    angles = []
    for is_retract in phases:
      if is_retract:
        angles.extend([retract_abduction_angle, retract_hip_angle,
                       retract_knee_angle])
      else:
        angles.extend([stance_abduction_angle, stance_hip_angle,
                       stance_knee_angle])
    return np.array(angles)

  def reset(self):
    pass

  def get_action(self, current_time, input_action):
    """Computes the trajectory according to input time and action.

    Args:
      current_time: The time in gym env since reset.
      input_action: A numpy array. The input leg pose from a NN controller.

    Returns:
      A numpy array. The desired motor angles.
    """

    return self._pose + self._alternating_legs_trajectory(
        current_time) + input_action

  def get_observation(self, input_observation):
    """Get the trajectory generator's observation."""

    return input_observation
