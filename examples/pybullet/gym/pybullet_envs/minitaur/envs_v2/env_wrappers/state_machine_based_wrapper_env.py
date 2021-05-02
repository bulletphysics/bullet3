"""A wrapped Quadruped with State-machine based controller."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import enum
import gin
import gym
import numpy as np

NUM_LEGS = 4
ACTION_DIM_COM = 2
ACTION_DIM_TOE = 1
ACTION_DIM_TOTAL = ACTION_DIM_COM + ACTION_DIM_TOE
OBSERVATION_DIM_LEG_ID = NUM_LEGS
OBSERVATION_DIM_TOE_POS = 2 * NUM_LEGS
OBSERVATION_DIM_TOTAL = OBSERVATION_DIM_TOE_POS + OBSERVATION_DIM_LEG_ID


# States of the state machine.
class GaitStateMachine(enum.IntEnum):
  """The state machine for quadruped gait."""
  STEP_LEFT_FRONT_TOE = 0
  STEP_RIGHT_HIND_TOE = 1
  STEP_RIGHT_FRONT_TOE = 2
  STEP_LEFT_HIND_TOE = 3
  TOTAL_GAIT_STATE_NUM = 4


@gin.configurable
class StateMachineBasedWrapperEnv(object):
  """An env using IK to convert toe positions to joint angles.

    The state machine consists of 4 states. During each state, the center of
    mass of the base link is moved first, then one of the legs will take a
    step by following a planned elliptical trajectory. The legs will move in
    the order of front left -> hind right -> front right -> hind left.
    The state transition is determined by elapsed time since last transition.
    Observation (16 dimensions):
    [one hot vector of the state id, local toe positions in x and y direction]
    Action (3 dimensions):
    [target moving distance of the current moving leg in x direction,
     target moving distance of base COM in x direction,
     target moving distance of base COM in y direction]
  """

  def __init__(self,
               gym_env,
               default_local_toe_positions,
               toe_link_indices=(3, 7, 11, 15),
               foot_lift_height=0.15,
               state_duration=2.0,
               action_lower_bound=(-0.0, -0.25, -0.25),
               action_upper_bound=(0.3, 0.25, 0.25),
               state_to_foot_id=(0, 3, 2, 1)):
    """Initialzes the wrapped env.

    Args:
      gym_env: An instance of LocomotionGymEnv.
      default_local_toe_positions: A list of vectors that contains the default
        local position of each toe.
      toe_link_indices: A list of indices to the toe link. Used for calculating
        local toe positions.
      foot_lift_height: Specifies how high the foot lifts during swing stage.
      state_duration: Specifies the duration of each state.
      action_lower_bound: Lower bound for the actions.
      action_upper_bound: Upper bound for the actions.
      state_to_foot_id: Mapping from state machine state to foot id.
    """
    self._gym_env = gym_env

    assert len(action_lower_bound) == ACTION_DIM_TOTAL
    assert len(action_upper_bound) == ACTION_DIM_TOTAL
    self.action_space = gym.spaces.Box(
        np.array(action_lower_bound), np.array(action_upper_bound))
    observation_lower_bound = np.array([-1.0] * OBSERVATION_DIM_TOTAL)
    observation_upper_bound = np.array([1.0] * OBSERVATION_DIM_TOTAL)
    self.observation_space = gym.spaces.Box(observation_lower_bound,
                                            observation_upper_bound)
    self._default_local_toe_positions = default_local_toe_positions
    self._toe_link_indices = toe_link_indices
    self._foot_lift_height = foot_lift_height
    self._state_to_foot_id = state_to_foot_id

    # Use the largest value for bounding the toe movement
    self._toe_move_bound = np.max(
        np.abs(np.concatenate([action_lower_bound, action_upper_bound])))

    # Duration of each state
    self.state_machine_state_duration = [state_duration] * int(
        GaitStateMachine.TOTAL_GAIT_STATE_NUM)

  def __getattr__(self, attr):
    return getattr(self._gym_env, attr)

  def _state_machine_observation(self):
    """Get the current observation: [state_id, local_toe_positions]."""
    observation = []
    # One-hot vector for the current state machine state.
    one_hot_foot_id = np.zeros(GaitStateMachine.TOTAL_GAIT_STATE_NUM)
    one_hot_foot_id[self.current_state_machine_state] = 1
    observation.extend(one_hot_foot_id)

    # Toe positions in X and Y direction in the local space.
    for toe_index in range(NUM_LEGS):
      toe_pos_local = self.current_local_toe_positions[toe_index]
      toe_pos_local_xy = [toe_pos_local[0], toe_pos_local[1]]
      observation.extend(toe_pos_local_xy)

    return observation

  def _get_constant_accel_interpolation(self, interp_ratio):
    """Modify an interpolation between 0 and 1 to have constant acceleration."""
    assert interp_ratio <= 1.0 and interp_ratio >= 0.0
    if interp_ratio < 0.5:
      return 0.5 * (2 * interp_ratio)**2
    else:
      return -0.5 * (2 * interp_ratio - 2)**2 + 1

  def _move_com(self, target_com_movement, time_since_transition,
                state_duration):
    """Get the ik action for moving the COM.

    Args:
      target_com_movement: Target COM movement relative to the previous COM
        position in the x-y plane.
      time_since_transition: Time elapsed since last state machien transition.
      state_duration: Duration of the state machine. The first half will be used
        for moving COM and the second half for moving the swing leg.

    Returns:
      ik_action: ik targets for moving the COM.
    """
    com_movement_ratio = np.clip(
        time_since_transition / ((state_duration - 0.0) / 2.0), 0, 1)
    com_movement_ratio = self._get_constant_accel_interpolation(
        com_movement_ratio)
    current_com_movement = np.array(target_com_movement) * com_movement_ratio

    ik_action = []
    for toe_index in range(NUM_LEGS):
      toe_pos_local = np.copy(self.current_local_toe_positions[toe_index])
      toe_pos_local[2] = self._default_local_toe_positions[toe_index][2]
      toe_pos_local[0] -= current_com_movement[0]
      toe_pos_local[1] -= current_com_movement[1]
      toe_pos_local[0] = np.clip(
          toe_pos_local[0], self._default_local_toe_positions[toe_index][0] -
          self._toe_move_bound,
          self._default_local_toe_positions[toe_index][0] +
          self._toe_move_bound)
      toe_pos_local[1] = np.clip(
          toe_pos_local[1], self._default_local_toe_positions[toe_index][1] -
          self._toe_move_bound,
          self._default_local_toe_positions[toe_index][1] +
          self._toe_move_bound)
      ik_action.extend(toe_pos_local)

    zero_translation = [0, 0, 0]
    identity_rotation = [0, 0, 0, 1]
    ik_action.extend(zero_translation)
    ik_action.extend(identity_rotation)

    return ik_action

  def _move_leg(self, target_toe_movement, time_since_transition,
                state_duration):
    """Get the ik action for moving the swing leg.

    Args:
      target_toe_movement: Target toe movement relative to the default toe
        position in the positive x direction.
      time_since_transition: Time elapsed since last state machien transition.
      state_duration: Duration of the state machine. The first half will be used
        for moving COM and the second half for moving the swing leg.

    Returns:
      ik_action: ik targets for moving the swing leg.
    """

    # The target toe position at the end of the movement.
    target_toe_local_position = np.array(self._default_local_toe_positions[
        self._state_to_foot_id[self.current_state_machine_state]])
    target_toe_local_position[0] += target_toe_movement

    # Toe position at the beginning of the movement.
    initial_toe_local_position = np.array(self.current_local_toe_positions[
        self._state_to_foot_id[self.current_state_machine_state]])

    # Auxiliary variables for computing the interpolation between the current
    # toe position and the target toe position.
    toe_circle_radius = 0.5 * np.linalg.norm(
        np.array(target_toe_local_position) - initial_toe_local_position)
    toe_moving_direction = np.array(
        target_toe_local_position) - initial_toe_local_position
    toe_moving_direction /= np.max([np.linalg.norm(toe_moving_direction), 1e-5])
    toe_traj_scale_ratio = self._foot_lift_height / np.max(
        [toe_circle_radius, 1e-5])

    # Current percentage of time into the state machine.
    toe_movement_ratio = np.clip(
        (time_since_transition - state_duration / 2.0) /
        ((state_duration - 0.0) / 2.0), 0, 1)
    toe_movement_ratio = self._get_constant_accel_interpolation(
        toe_movement_ratio)
    toe_circle_moved_angle = np.pi * toe_movement_ratio

    # Target horizontal movement from the previous local toe position.
    target_toe_horizontal_movement = toe_circle_radius - np.cos(
        toe_circle_moved_angle) * toe_circle_radius
    current_toe_target = np.array([
        target_toe_horizontal_movement * toe_moving_direction[0],
        target_toe_horizontal_movement * toe_moving_direction[1],
        np.sin(toe_circle_moved_angle) * toe_circle_radius *
        toe_traj_scale_ratio
    ]) + initial_toe_local_position

    ik_action = []
    for toe_index in range(NUM_LEGS):
      toe_pos_local = np.copy(self.current_local_toe_positions[toe_index])
      toe_pos_local[2] = self._default_local_toe_positions[toe_index][2]
      if toe_index == self._state_to_foot_id[self.current_state_machine_state]:
        toe_pos_local[0] = current_toe_target[0]
        toe_pos_local[1] = current_toe_target[1]
        toe_pos_local[2] = current_toe_target[2]

      ik_action.extend(toe_pos_local)

    zero_translation = [0, 0, 0]
    identity_rotation = [0, 0, 0, 1]
    ik_action.extend(zero_translation)
    ik_action.extend(identity_rotation)
    return ik_action

  def _update_state_machine_transition(self):
    """Update the state machine state if the duration has been reached."""
    self.current_state_machine_state = (self.current_state_machine_state + 1
                                       ) % GaitStateMachine.TOTAL_GAIT_STATE_NUM
    self.last_state_transition_time = self.robot.GetTimeSinceReset()

  def _update_local_toe_positions(self):
    """Update the local position of the toes."""
    identity_orientation = [0, 0, 0, 1]
    base_position = self.robot.GetBasePosition()
    base_orientation = self.robot.GetBaseOrientation()
    inv_base_position, inv_base_orientation = self.pybullet_client.invertTransform(
        base_position, base_orientation)
    for toe_index in range(NUM_LEGS):
      toe_pose_world = self.pybullet_client.getLinkState(
          self.robot.quadruped, self._toe_link_indices[toe_index])[0]
      toe_pos_local, _ = self.pybullet_client.multiplyTransforms(
          inv_base_position, inv_base_orientation, toe_pose_world,
          identity_orientation)
      self.current_local_toe_positions[toe_index] = np.array(toe_pos_local)

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
    action = np.clip(action, self.action_space.low, self.action_space.high)
    sum_reward = 0
    step_num = 0
    state_duration = self.state_machine_state_duration[
        self.current_state_machine_state]
    time_since_transition = self.robot.GetTimeSinceReset(
    ) - self.last_state_transition_time

    # Move COM
    while time_since_transition < state_duration / 2.0:
      ik_actions = self._move_com(action[1:3], time_since_transition,
                                  state_duration)
      _, reward, done, _ = self._gym_env.step(ik_actions)
      sum_reward += reward
      step_num += 1
      time_since_transition = self.robot.GetTimeSinceReset(
      ) - self.last_state_transition_time
      if done:
        break
    self._update_local_toe_positions()
    # Move Leg
    while time_since_transition < state_duration:
      ik_actions = self._move_leg(action[0], time_since_transition,
                                  state_duration)
      _, reward, done, _ = self._gym_env.step(ik_actions)
      sum_reward += reward
      step_num += 1
      time_since_transition = self.robot.GetTimeSinceReset(
      ) - self.last_state_transition_time
      if done:
        break
    self._update_local_toe_positions()
    self._update_state_machine_transition()

    state_machine_observation = self._state_machine_observation()

    return state_machine_observation, sum_reward, done, _

  def reset(self):
    """Reset the simulation and state machine states."""
    self.current_state_machine_state = GaitStateMachine.STEP_LEFT_FRONT_TOE
    self.last_state_transition_time = 0
    self.current_local_toe_positions = np.copy(
        self._default_local_toe_positions)

    self._gym_env.reset()

    state_machine_observation = self._state_machine_observation()

    return state_machine_observation
