"""Extends the environment by adding observation and action history.

The implementation is a bit dirty import of the implementation in
the experimental branch.

"""

from gym import spaces
import numpy as np

from pybullet_envs.minitaur.envs.minitaur_reactive_env import MinitaurReactiveEnv


class MinitaurExtendedEnv(MinitaurReactiveEnv):
  """The 'extended' environment for Markovian property.

  This class implements to include prior actions and observations to the
  observation vector, thus making the environment "more" Markovian. This is
  especially useful for systems with latencies.

  Args:
    history_length: the length of the historic data
    history_include_actions: a flag for including actions as history
    history_include_states: a flag for including states as history
    include_state_difference: a flag for including the first-order differences
      as history
    include_second_state_difference: a flag for including the second-order state
      differences as history.
    include_base_position: a flag for including the base as observation,
    never_terminate: if this is on, the environment unconditionally never
      terminates.
    action_scale: the scale of actions,
  """
  MAX_BUFFER_SIZE = 1001
  ACTION_DIM = 8
  PARENT_OBSERVATION_DIM = 12
  INIT_EXTENSION_POS = 2.0
  INIT_SWING_POS = 0.0

  metadata = {
      "render.modes": ["human", "rgb_array"],
      "video.frames_per_second": 50,
  }

  def __init__(self,
               history_length=1,
               history_include_actions=True,
               history_include_states=False,
               include_state_difference=False,
               include_second_state_difference=False,
               include_base_position=False,
               include_leg_model=False,
               never_terminate=False,
               action_scale=0.5,
               **kwargs):
    self._kwargs = kwargs

    self._history_length = history_length
    self._history_include_actions = history_include_actions
    self._history_include_states = history_include_states
    self._include_state_difference = include_state_difference
    self._include_second_state_difference = include_second_state_difference
    self._include_base_position = include_base_position
    self._include_leg_model = include_leg_model

    self._never_terminate = never_terminate
    self._action_scale = action_scale

    self._past_parent_observations = np.zeros((self.MAX_BUFFER_SIZE + 1,
                                               self.PARENT_OBSERVATION_DIM))
    self._past_motor_angles = np.zeros((self.MAX_BUFFER_SIZE + 1, 8))
    self._past_actions = np.zeros((self.MAX_BUFFER_SIZE, self.ACTION_DIM))
    self._counter = 0

    super(MinitaurExtendedEnv, self).__init__(**kwargs)
    self.action_space = spaces.Box(-1.0, 1.0, self.action_space.shape)
    self.observation_space = spaces.Box(-np.inf, np.inf,
                                        self._get_observation().shape)
    # This is mainly for the TF-Agents compatibility
    self.action_space.flat_dim = len(self.action_space.low)
    self.observation_space.flat_dim = len(self.observation_space.low)

  def _get_observation(self):
    """Maybe concatenate motor velocity and torque into observations."""
    parent_observation = super(MinitaurExtendedEnv, self)._get_observation()
    parent_observation = np.array(parent_observation)
    # Base class might require this.
    self._observation = parent_observation
    self._past_parent_observations[self._counter] = parent_observation
    num_motors = self.minitaur.num_motors
    self._past_motor_angles[self._counter] = parent_observation[-num_motors:]

    history_states = []
    history_actions = []
    for i in range(self._history_length):
      t = max(self._counter - i - 1, 0)

      if self._history_include_states:
        history_states.append(self._past_parent_observations[t])

      if self._history_include_actions:
        history_actions.append(self._past_actions[t])

    t = self._counter
    tm, tmm = max(0, self._counter - 1), max(0, self._counter - 2)

    state_difference, second_state_difference = [], []
    if self._include_state_difference:
      state_difference = [
          self._past_motor_angles[t] - self._past_motor_angles[tm]
      ]
    if self._include_second_state_difference:
      second_state_difference = [
          self._past_motor_angles[t] - 2 * self._past_motor_angles[tm] +
          self._past_motor_angles[tmm]
      ]

    base_position = []
    if self._include_base_position:
      base_position = np.array((self.minitaur.GetBasePosition()))

    leg_model = []
    if self._include_leg_model:
      raw_motor_angles = self.minitaur.GetMotorAngles()
      leg_model = self.convert_to_leg_model(raw_motor_angles)

    observation_list = (
        [parent_observation] + history_states + history_actions +
        state_difference + second_state_difference + [base_position] +
        [leg_model])

    full_observation = np.concatenate(observation_list)
    return full_observation

  def reset(self):
    """Resets the time and history buffer."""
    self._counter = 0
    self._signal(self._counter)  # This sets the current phase
    self._past_parent_observations = np.zeros((self.MAX_BUFFER_SIZE + 1,
                                               self.PARENT_OBSERVATION_DIM))
    self._past_motor_angles = np.zeros((self.MAX_BUFFER_SIZE + 1, 8))
    self._past_actions = np.zeros((self.MAX_BUFFER_SIZE, self.ACTION_DIM))
    self._counter = 0

    return np.array(super(MinitaurExtendedEnv, self).reset())

  def step(self, action):
    """Step function wrapper can be used to add shaping terms to the reward.

    Args:
      action: an array of the given action

    Returns:
      next_obs: the next observation
      reward: the reward for this experience tuple
      done: the terminal flag
      info: an additional information
    """

    action *= self._action_scale

    self._past_actions[self._counter] = action
    self._counter += 1

    next_obs, _, done, info = super(MinitaurExtendedEnv, self).step(action)

    reward = self.reward()
    info.update(base_reward=reward)

    return next_obs, reward, done, info

  def terminate(self):
    """The helper function to terminate the environment."""
    super(MinitaurExtendedEnv, self)._close()

  def _termination(self):
    """Determines whether the env is terminated or not.

    checks whether 1) the front leg is bent too much or 2) the time exceeds
    the manually set weights.

    Returns:
      terminal: the terminal flag whether the env is terminated or not
    """
    if self._never_terminate:
      return False

    leg_model = self.convert_to_leg_model(self.minitaur.GetMotorAngles())
    swing0 = leg_model[0]
    swing1 = leg_model[2]
    maximum_swing_angle = 0.8
    if swing0 > maximum_swing_angle or swing1 > maximum_swing_angle:
      return True

    if self._counter >= 500:
      return True

    return False

  def reward(self):
    """Compute rewards for the given time step.

    It considers two terms: 1) forward velocity reward and 2) action
    acceleration penalty.

    Returns:
      reward: the computed reward.
    """
    current_base_position = self.minitaur.GetBasePosition()
    dt = self.control_time_step
    velocity = (current_base_position[0] - self._last_base_position[0]) / dt
    velocity_reward = np.clip(velocity, -0.5, 0.5)

    action = self._past_actions[self._counter - 1]
    prev_action = self._past_actions[max(self._counter - 2, 0)]
    prev_prev_action = self._past_actions[max(self._counter - 3, 0)]
    acc = action - 2 * prev_action + prev_prev_action
    action_acceleration_penalty = np.mean(np.abs(acc))

    reward = 0.0
    reward += 1.0 * velocity_reward
    reward -= 0.1 * action_acceleration_penalty

    return reward

  @staticmethod
  def convert_to_leg_model(motor_angles):
    """A helper function to convert motor angles to leg model.

    Args:
      motor_angles: raw motor angles:

    Returns:
      leg_angles: the leg pose model represented in swing and extension.
    """
    # TODO(sehoonha): clean up model conversion codes
    num_legs = 4
    # motor_angles = motor_angles / (np.pi / 4.)
    leg_angles = np.zeros(num_legs * 2)
    for i in range(num_legs):
      motor1, motor2 = motor_angles[2 * i:2 * i + 2]
      swing = (-1)**(i // 2) * 0.5 * (motor2 - motor1)
      extension = 0.5 * (motor1 + motor2)

      leg_angles[i] = swing
      leg_angles[i + num_legs] = extension

    return leg_angles

  def __getstate__(self):
    """A helper get state function for pickling."""
    return {"kwargs": self._kwargs}

  def __setstate__(self, state):
    """A helper set state function for pickling."""
    self.__init__(**state["kwargs"])
