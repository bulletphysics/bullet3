"""A wrapped MinitaurGymEnv with a built-in controller."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
import attr
from gym import spaces
import numpy as np
import gin
from pybullet_envs.minitaur.agents.trajectory_generator import tg_inplace
from pybullet_envs.minitaur.envs.utilities import laikago_pose_utils
from pybullet_envs.minitaur.envs.utilities import minitaur_pose_utils

_NUM_LEGS = 4
_LAIKAGO_NUM_ACTIONS = 12
_FREQ_LOWER_BOUND = 0.0
_FREQ_UPPER_BOUND = 3
_DEFAULT_AMPLITUDE_STANCE = -0.02
_DEFAULT_AMPLITUDE_LIFT = 0.9
_DEFAULT_CENTER_EXTENSION = 0
_DEFAULT_STANCE_LIFT_CUTOFF = 2 * np.pi * 0.67
_DEFAULT_RESIDUAL_RANGE = 0.4
_LAIKAGO_KNEE_ACTION_INDEXES = [2, 5, 8, 11]
MINITAUR_INIT_EXTENSION_POS = 2.0
MINITAUR_INIT_SWING_POS = 0.0


@gin.configurable
class PmtgInplaceWrapperEnv(object):
  """A wrapped LocomotionGymEnv with a built-in trajectory generator."""

  def __init__(self,
               gym_env,
               freq_lower_bound=_FREQ_LOWER_BOUND,
               freq_upper_bound=_FREQ_UPPER_BOUND,
               residual_range=_DEFAULT_RESIDUAL_RANGE,
               amplitude_stance=_DEFAULT_AMPLITUDE_STANCE,
               amplitude_lift=_DEFAULT_AMPLITUDE_LIFT,
               center_extension=_DEFAULT_CENTER_EXTENSION,
               stance_lift_cutoff=_DEFAULT_STANCE_LIFT_CUTOFF):
    """Initializes the TG inplace wrapper class.

    Args:
      gym_env: the gym environment to wrap on.
      freq_lower_bound: minimum frequency that the TGs can be propagated at.
      freq_upper_bound: maximum frequency that the TGs can be propagated at.
      residual_range: range of residuals that can be added to tg outputs.
      amplitude_stance: stance amplitude of TG (see tg_inplace.py for details).
      amplitude_lift: swing amplitude of TG (see tg_inplace.py for details).
      center_extension: center extension of TG (see tg_inplace.py for details).
      stance_lift_cutoff: phase cutoff between stance and lift phase (see
        tg_inplace.py for details).
    """
    self._gym_env = gym_env
    self._num_actions = gym_env.robot.num_motors
    self._tg_phases = tg_inplace.reset()
    self._tg_params = dict(
        amplitude_stance=amplitude_stance,
        amplitude_lift=amplitude_lift,
        center_extension=center_extension,
        stance_lift_cutoff=stance_lift_cutoff)

    # Add the action boundaries for delta time, one per integrator.
    action_low = np.hstack(
        ([-residual_range] * self._num_actions, [freq_lower_bound] * _NUM_LEGS))
    action_high = np.hstack(
        ([residual_range] * self._num_actions, [freq_upper_bound] * _NUM_LEGS))
    self.action_space = spaces.Box(action_low, action_high)

    # Set the observation space and boundaries.
    if hasattr(self._gym_env.observation_space, "spaces"):
      self.observation_space = self._gym_env.observation_space
      self.observation_space.spaces["pmtg_inplace"] = spaces.Box(
          -1 * np.ones(2 * _NUM_LEGS), np.ones(2 * _NUM_LEGS))
    else:
      lower_bound = self._gym_env.observation_space.low
      upper_bound = self._gym_env.observation_space.high
      lower_bound = np.hstack((lower_bound, [-1.] * 2 * _NUM_LEGS))
      upper_bound = np.hstack((upper_bound, [1.] * 2 * _NUM_LEGS))
      self.observation_space = spaces.Box(lower_bound, upper_bound)

  def __getattr__(self, attrb):
    return getattr(self._gym_env, attrb)

  def _modify_observation(self, observation):
    if isinstance(observation, dict):
      observation["tg_inplace"] = np.hstack(
          (np.cos(self._tg_phases), np.sin(self._tg_phases)))
      return observation
    else:
      return np.hstack(
          (observation, np.cos(self._tg_phases), np.sin(self._tg_phases)))

  def reset(self, initial_motor_angles=None, reset_duration=1.0):
    """Resets the environment as well as trajectory generators."""
    self._last_real_time = 0
    self._num_step = 0
    self._tg_phases = tg_inplace.reset()
    if self._num_actions == _LAIKAGO_NUM_ACTIONS:
      # Use laikago's init pose as zero action.
      init_pose = np.array(
          attr.astuple(
              laikago_pose_utils.LaikagoPose(
                  abduction_angle_0=laikago_pose_utils
                  .LAIKAGO_DEFAULT_ABDUCTION_ANGLE,
                  hip_angle_0=laikago_pose_utils.LAIKAGO_DEFAULT_HIP_ANGLE,
                  knee_angle_0=laikago_pose_utils.LAIKAGO_DEFAULT_KNEE_ANGLE,
                  abduction_angle_1=laikago_pose_utils
                  .LAIKAGO_DEFAULT_ABDUCTION_ANGLE,
                  hip_angle_1=laikago_pose_utils.LAIKAGO_DEFAULT_HIP_ANGLE,
                  knee_angle_1=laikago_pose_utils.LAIKAGO_DEFAULT_KNEE_ANGLE,
                  abduction_angle_2=laikago_pose_utils
                  .LAIKAGO_DEFAULT_ABDUCTION_ANGLE,
                  hip_angle_2=laikago_pose_utils.LAIKAGO_DEFAULT_HIP_ANGLE,
                  knee_angle_2=laikago_pose_utils.LAIKAGO_DEFAULT_KNEE_ANGLE,
                  abduction_angle_3=laikago_pose_utils
                  .LAIKAGO_DEFAULT_ABDUCTION_ANGLE,
                  hip_angle_3=laikago_pose_utils.LAIKAGO_DEFAULT_HIP_ANGLE,
                  knee_angle_3=laikago_pose_utils.LAIKAGO_DEFAULT_KNEE_ANGLE)))
      self._init_pose = init_pose
      observation = self._gym_env.reset(init_pose, reset_duration)
    else:
      # Use minitaur's init pose as zero action.
      init_pose = np.array(
          attr.astuple(
              minitaur_pose_utils.MinitaurPose(
                  swing_angle_0=MINITAUR_INIT_SWING_POS,
                  swing_angle_1=MINITAUR_INIT_SWING_POS,
                  swing_angle_2=MINITAUR_INIT_SWING_POS,
                  swing_angle_3=MINITAUR_INIT_SWING_POS,
                  extension_angle_0=MINITAUR_INIT_EXTENSION_POS,
                  extension_angle_1=MINITAUR_INIT_EXTENSION_POS,
                  extension_angle_2=MINITAUR_INIT_EXTENSION_POS,
                  extension_angle_3=MINITAUR_INIT_EXTENSION_POS)))
      initial_motor_angles = minitaur_pose_utils.leg_pose_to_motor_angles(
          init_pose)
      observation = self._gym_env.reset(initial_motor_angles, reset_duration)
    return self._modify_observation(observation)

  def step(self, action):
    """Steps the wrapped PMTG inplace environment."""
    time = self._gym_env.get_time_since_reset()

    # Convert the policy's residual actions to motor space.
    if self._num_actions == _LAIKAGO_NUM_ACTIONS:
      action_residual = np.array(
          attr.astuple(
              laikago_pose_utils.LaikagoPose(*(action[0:self._num_actions]))))
    else:
      action_residual = minitaur_pose_utils.leg_pose_to_motor_angles(
          action[0:self._num_actions])

    self._last_real_time = time
    self._tg_phases, tg_extensions = tg_inplace.step(
        self._tg_phases, action[-_NUM_LEGS:], self._gym_env.env_time_step,
        self._tg_params)
    # Convert TG's actions to motor space depending on the robot type.
    if self._num_actions == _LAIKAGO_NUM_ACTIONS:
      # If the legs have 3 DOF, apply extension directly to knee joints
      action_tg_motor_space = np.zeros(self._num_actions)
      for tg_idx, knee_idx in zip(
          range(_NUM_LEGS), _LAIKAGO_KNEE_ACTION_INDEXES):
        action_tg_motor_space[knee_idx] = tg_extensions[tg_idx]
    else:
      # Conversion to motor space for minitaur robot.
      action_tg_motor_space = []
      for idx in range(_NUM_LEGS):
        extend = tg_extensions[idx]
        action_tg_motor_space.extend(
            minitaur_pose_utils.swing_extend_to_motor_angles(idx, 0, extend))
    new_action = action_tg_motor_space + action_residual
    if self._num_actions == _LAIKAGO_NUM_ACTIONS:
      new_action += self._init_pose
    original_observation, reward, done, _ = self._gym_env.step(new_action)

    return self._modify_observation(original_observation), reward, done, _
