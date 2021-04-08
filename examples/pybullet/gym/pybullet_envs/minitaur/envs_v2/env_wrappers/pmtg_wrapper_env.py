"""A wrapped MinitaurGymEnv with a built-in controller."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from gym import spaces
import numpy as np
import gin
from pybullet_envs.minitaur.agents.trajectory_generator import tg_simple
from pybullet_envs.minitaur.envs_v2.utilities import robot_pose_utils
from pybullet_envs.minitaur.robots.utilities import action_filter

_DELTA_TIME_LOWER_BOUND = 0.0
_DELTA_TIME_UPPER_BOUND = 3

_GAIT_PHASE_MAP = {
    "walk": [0, 0.25, 0.5, 0.75],
    "trot": [0, 0.5, 0.5, 0],
    "bound": [0, 0.5, 0, 0.5],
    "pace": [0, 0, 0.5, 0.5],
    "pronk": [0, 0, 0, 0]
}


@gin.configurable
class PmtgWrapperEnv(object):
  """A wrapped LocomotionGymEnv with a built-in trajectory generator."""

  def __init__(self,
               gym_env,
               intensity_upper_bound=1.5,
               min_delta_time=_DELTA_TIME_LOWER_BOUND,
               max_delta_time=_DELTA_TIME_UPPER_BOUND,
               integrator_coupling_mode="all coupled",
               walk_height_coupling_mode="all coupled",
               variable_swing_stance_ratio=True,
               swing_stance_ratio=1.0,
               residual_range=0.15,
               init_leg_phase_offsets=None,
               init_gait=None,
               default_walk_height=0,
               action_filter_enable=True,
               action_filter_order=1,
               action_filter_low_cut=0,
               action_filter_high_cut=3.0,
               action_filter_initialize=False,
               leg_pose_class=None):
    """Initialzes the wrapped env.

    Args:
      gym_env: An instance of LocomotionGymEnv.
      intensity_upper_bound: The upper bound for intensity of the trajectory
        generator. It can be used to limit the leg movement.
      min_delta_time: Lower limit for the time in seconds that the trajectory
        generator can be moved forward at each simulation step. The effective
        frequency of the gait is based on the delta time multiplied by the
        internal frequency of the trajectory generator.
      max_delta_time: Upper limit for the time in seconds that the trajectory
        generator can be moved forward at each simulation step.
      integrator_coupling_mode: How the legs should be coupled for integrators.
      walk_height_coupling_mode: The same coupling mode used for walking walking
        heights for the legs.
      variable_swing_stance_ratio: A boolean to indicate if the swing stance
        ratio can change per time step or not.
      swing_stance_ratio: Time taken by swing phase vs stance phase. This is
        only relevant if variable_swing_stance_ratio is False.
      residual_range: The upper limit for the residual actions that adds to the
        leg motion. It is 0.15 by default, can be increased for more flexibility
        or decreased to only to use the trajectory generator's motion.
      init_leg_phase_offsets: The initial phases of the legs. A list of 4
        variables within [0,1). The order is front-left, rear-left, front-right
        and rear-right.
      init_gait: The initial gait that sets the starting phase difference
        between the legs. Overrides the arg init_phase_offsets. Has to be
        "walk", "trot", "bound" or "pronk". Used in vizier search.
      default_walk_height: Offset for the extension of the legs for the robot.
        Applied to the legs at every time step. Implicitly affects the walking
        and standing height of the policy. Zero by default. Units is in
        extension space (can be considered in radiant since it is a linear
        transformation to motor angles based on the robot's geometry).
      action_filter_enable: Use a butterworth filter for the output of the PMTG
        actions (before conversion to leg swing-extend model). It forces
        smoother behaviors depending on the parameters used.
      action_filter_order: The order for the action_filter (1 by default).
      action_filter_low_cut: The cut for the lower frequencies (0 by default).
      action_filter_high_cut: The cut for the higher frequencies (3 by default).
      action_filter_initialize: If the action filter should be initialized when
        the first action is taken. If enabled, the filter does not affect action
        value the first time it is called and fills the history with that value.
      leg_pose_class: A class providing a convert_leg_pose_to_motor_angle
        instance method or None. If None, robot_pose_utils is used.

    Raises:
      ValueError if the controller does not implement get_action and
      get_observation.

    """
    self._gym_env = gym_env
    self._num_actions = gym_env.robot.num_motors
    self._residual_range = residual_range
    self._min_delta_time = min_delta_time
    self._max_delta_time = max_delta_time
    self._leg_pose_util = leg_pose_class() if leg_pose_class else None
    # If not specified, default leg phase offsets to walking.
    if init_gait:
      if init_gait in _GAIT_PHASE_MAP:
        init_leg_phase_offsets = _GAIT_PHASE_MAP[init_gait]
      else:
        raise ValueError("init_gait is not one of the defined gaits.")
    else:
      init_leg_phase_offsets = init_leg_phase_offsets or [0, 0.25, 0.5, 0.75]
    # Create the Trajectory Generator based on the parameters.
    self._trajectory_generator = tg_simple.TgSimple(
        intensity_upper_bound=intensity_upper_bound,
        integrator_coupling_mode=integrator_coupling_mode,
        walk_height_coupling_mode=walk_height_coupling_mode,
        variable_swing_stance_ratio=variable_swing_stance_ratio,
        swing_stance_ratio=swing_stance_ratio,
        init_leg_phase_offsets=init_leg_phase_offsets)

    action_dim = self._extend_action_space()
    self._extend_obs_space()

    self._default_walk_height = default_walk_height
    self._action_filter_enable = action_filter_enable
    if self._action_filter_enable:
      self._action_filter_initialize = action_filter_initialize
      self._action_filter_order = action_filter_order
      self._action_filter_low_cut = action_filter_low_cut
      self._action_filter_high_cut = action_filter_high_cut
      self._action_filter = self._build_action_filter(action_dim)

  def _extend_obs_space(self):
    """Extend observation space to include pmtg phase variables."""
    # Set the observation space and boundaries.
    lower_bound, upper_bound = self._get_observation_bounds()
    if hasattr(self._gym_env.observation_space, "spaces"):
      new_obs_space = spaces.Box(np.array(lower_bound), np.array(upper_bound))
      self.observation_space.spaces.update({"pmtg_phase": new_obs_space})
    else:
      lower_bound = np.append(self._gym_env.observation_space.low, lower_bound)
      upper_bound = np.append(self._gym_env.observation_space.high, upper_bound)
      self.observation_space = spaces.Box(
          np.array(lower_bound), np.array(upper_bound), dtype=np.float32)

  def _extend_action_space(self):
    """Extend the action space to include pmtg parameters."""
    # Add the action boundaries for delta time, one per integrator.
    action_low = [-self._residual_range] * self._num_actions
    action_high = [self._residual_range] * self._num_actions
    action_low = np.append(action_low, [self._min_delta_time] *
                           self._trajectory_generator.num_integrators)
    action_high = np.append(action_high, [self._max_delta_time] *
                            self._trajectory_generator.num_integrators)

    # Add the action boundaries for parameters of the trajectory generator.
    l_bound, u_bound = self._trajectory_generator.get_parameter_bounds()
    action_low = np.append(action_low, l_bound)
    action_high = np.append(action_high, u_bound)
    self.action_space = spaces.Box(
        np.array(action_low), np.array(action_high), dtype=np.float32)
    return len(action_high)

  def __getattr__(self, attrb):
    return getattr(self._gym_env, attrb)

  def _modify_observation(self, observation):
    if isinstance(observation, dict):
      observation["pmtg_phase"] = self._trajectory_generator.get_state()
      return observation
    else:
      return np.append(observation, self._trajectory_generator.get_state())

  def reset(self, initial_motor_angles=None, reset_duration=1.0):
    """Resets the environment as well as the trajectory generator(s).

    Args:
      initial_motor_angles: Unused for PMTG. Instead, it sets the legs to a pose
        with the neutral action for the trajectory generator.
      reset_duration: Float. The time (in seconds) needed to rotate all motors
        to the desired initial values.

    Returns:
      A numpy array contains the initial observation after reset.
    """
    del initial_motor_angles
    if self._action_filter_enable:
      self._reset_action_filter()
    self._last_real_time = 0
    self._num_step = 0
    self._target_speed_coef = 0.0
    if self._trajectory_generator:
      self._trajectory_generator.reset()
    if self._leg_pose_util:
      initial_motor_angles = self._leg_pose_util.convert_leg_pose_to_motor_angles(
          [0, 0, 0] * 4)
    else:
      initial_motor_angles = robot_pose_utils.convert_leg_pose_to_motor_angles(
          self._gym_env.robot_class, [0, 0, 0] * 4)
    observation = self._gym_env.reset(initial_motor_angles, reset_duration)
    return self._modify_observation(observation)

  def step(self, action):
    """Steps the wrapped environment.

    Args:
      action: Numpy array. The input action from an NN agent.

    Returns:
      The tuple containing the modified observation, the reward, the epsiode end
      indicator.

    Raises:
      ValueError if input action is None.

    """

    if action is None:
      raise ValueError("Action cannot be None")

    if self._action_filter_enable:
      action = self._filter_action(action)

    time = self._gym_env.get_time_since_reset()

    action_residual = action[0:self._num_actions]
    # Add the default walking height offset to extension.
    dimensions = len(action_residual) // 4
    action_residual[dimensions - 1::dimensions] += self._default_walk_height
    # Calculate trajectory generator's output based on the rest of the actions.
    delta_real_time = time - self._last_real_time
    self._last_real_time = time
    action_tg = self._trajectory_generator.get_actions(
        delta_real_time, action[self._num_actions:])
    # If the residuals have a larger dimension, extend trajectory generator's
    # actions to include abduction motors.
    if len(action_tg) == 8 and len(action_residual) == 12:
      for i in [0, 3, 6, 9]:
        action_tg.insert(i, 0)
    # Add TG actions with residual actions (in swing - extend space).
    action_total = [a + b for a, b in zip(action_tg, action_residual)]
    # Convert them to motor space based on the robot-specific conversions.
    if self._leg_pose_util:
      action_motor_space = self._leg_pose_util.convert_leg_pose_to_motor_angles(
          action_total)
    else:
      action_motor_space = robot_pose_utils.convert_leg_pose_to_motor_angles(
          self._gym_env.robot_class, action_total)
    original_observation, reward, done, _ = self._gym_env.step(
        action_motor_space)

    return self._modify_observation(original_observation), np.float32(
        reward), done, _

  def _get_observation_bounds(self):
    """Get the bounds of the observation added from the trajectory generator.

    Returns:
      lower_bounds: Lower bounds for observations.
      upper_bounds: Upper bounds for observations.
    """
    lower_bounds = self._trajectory_generator.get_state_lower_bounds()
    upper_bounds = self._trajectory_generator.get_state_upper_bounds()
    return lower_bounds, upper_bounds

  def _build_action_filter(self, num_joints):
    order = self._action_filter_order
    low_cut = self._action_filter_low_cut
    high_cut = self._action_filter_high_cut
    sampling_rate = 1 / (0.01)
    a_filter = action_filter.ActionFilterButter([low_cut], [high_cut],
                                                sampling_rate, order,
                                                num_joints)
    return a_filter

  def _reset_action_filter(self):
    self._action_filter.reset()
    self._action_filter_empty = True
    return

  def _filter_action(self, action):
    if self._action_filter_empty and self._action_filter_initialize:
      # If initialize is selected and it is the first time filter is called,
      # fill the buffer with that action so that it starts from that value
      # instead of zero(s).
      init_action = np.array(action).reshape(len(action), 1)
      self._action_filter.reset(init_action)
      self._action_filter_empty = False
    filtered_action = self._action_filter.filter(np.array(action))
    return filtered_action
