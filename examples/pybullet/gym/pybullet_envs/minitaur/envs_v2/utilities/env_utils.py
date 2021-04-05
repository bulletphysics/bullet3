"""Utility functions to manipulate environment."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import collections
import gin
from gym import spaces
import numpy as np

import tensorflow.compat.v1 as tf




def flatten_observations(observation_dict, observation_excluded=()):
  """Flattens the observation dictionary to an array.

  If observation_excluded is passed in, it will still return a dictionary,
  which includes all the (key, observation_dict[key]) in observation_excluded,
  and ('other': the flattened array).

  Args:
    observation_dict: A dictionary of all the observations.
    observation_excluded: A list/tuple of all the keys of the observations to be
      ignored during flattening.

  Returns:
    An array or a dictionary of observations based on whether
      observation_excluded is empty.
  """
  if not isinstance(observation_excluded, (list, tuple)):
    observation_excluded = [observation_excluded]
  observations = []
  for key, value in observation_dict.items():
    if key not in observation_excluded:
      observations.append(np.asarray(value).flatten())
  flat_observations = np.float32(np.concatenate(observations))
  if not observation_excluded:
    return flat_observations
  else:
    observation_dict_after_flatten = {"other": flat_observations}
    for key in observation_excluded:
      if key in observation_dict:
        observation_dict_after_flatten[key] = observation_dict[key]
    return collections.OrderedDict(
        sorted(list(observation_dict_after_flatten.items())))


def flatten_observation_dim(observation_dim, observation_excluded=()):
  """Flattens the observation dimensions to an array.

  If observation_excluded is passed in, it will still return a dictionary,
  which includes all the (key, observation_dict[key]) in observation_excluded,
  and ('other': the flattened array).

  Args:
    observation_dim: A dictionary of all the observation dimensions.
    observation_excluded: A list/tuple of all the keys of the observations to be
      ignored during flattening.

  Returns:
    An array or a dictionary of observation dimensions based on whether
      observation_excluded is empty.
  """
  if not isinstance(observation_excluded, (list, tuple)):
    observation_excluded = [observation_excluded]
  observation_dims = 0
  for key, value in observation_dim.items():
    if key not in observation_excluded:
      observation_dims += value
  if not observation_excluded:
    return observation_dims
  else:
    dim_dict_after_flatten = {"other": observation_dims}
    for key in observation_excluded:
      if key in observation_dim:
        dim_dict_after_flatten[key] = observation_dim[key]
    return collections.OrderedDict(sorted(list(dim_dict_after_flatten.items())))


def flatten_observation_spaces(observation_spaces, observation_excluded=()):
  """Flattens the dictionary observation spaces to gym.spaces.Box.

  If observation_excluded is passed in, it will still return a dictionary,
  which includes all the (key, observation_spaces[key]) in observation_excluded,
  and ('other': the flattened Box space).

  Args:
    observation_spaces: A dictionary of all the observation spaces.
    observation_excluded: A list/tuple of all the keys of the observations to be
      ignored during flattening.

  Returns:
    A box space or a dictionary of observation spaces based on whether
      observation_excluded is empty.
  """
  if not isinstance(observation_excluded, (list, tuple)):
    observation_excluded = [observation_excluded]
  lower_bound = []
  upper_bound = []
  for key, value in observation_spaces.spaces.items():
    if key not in observation_excluded:
      lower_bound.append(np.asarray(value.low).flatten())
      upper_bound.append(np.asarray(value.high).flatten())
  lower_bound = np.concatenate(lower_bound)
  upper_bound = np.concatenate(upper_bound)
  observation_space = spaces.Box(
      np.array(lower_bound), np.array(upper_bound), dtype=np.float32)
  if not observation_excluded:
    return observation_space
  else:
    observation_spaces_after_flatten = {"other": observation_space}
    for key in observation_excluded:
      if key in observation_spaces.spaces:
        observation_spaces_after_flatten[key] = observation_spaces[key]
    return spaces.Dict(observation_spaces_after_flatten)



@gin.configurable
def get_action_spec(action_spec):
  """Get action spec for one agent from the environment specs."""
  return list(action_spec.values())[0]



@gin.configurable
def get_get_actions_fn(agent_name_to_index):
  """Get function which returns other agents' actions."""

  def get_actions(action):
    """Returns a list of actions.

    Args:
      action: A dictionary of action tensors with keys matching agent names

    Returns:
      critic_actions: A list of action tensors for (N-1) other agents.
        Shape: (B x N, N-1, D)
    """
    critic_actions = []
    for agent_name in sorted(agent_name_to_index.keys()):
      other_agent_actions = []
      for other_agent_name in sorted(agent_name_to_index.keys()):
        if other_agent_name != agent_name:
          other_agent_actions.append(action[other_agent_name])
      critic_actions.append(other_agent_actions)
    print([tf.shape(critic_action) for critic_action in critic_actions])
    # Shape goes from (N, N-1, B, D) to (B x N, N-1, D)
    critic_actions = tf.transpose(tf.concat(critic_actions, axis=1), (1, 0, 2))
    return critic_actions

  return get_actions




def get_robot_base_position(robot):
  """Gets the base position of robot."""
  # TODO(b/151975607): Clean this after robot interface migration.
  if hasattr(robot, "GetBasePosition"):
    return robot.GetBasePosition()
  else:
    return robot.base_position


def get_robot_base_orientation(robot):
  """Gets the base orientation of robot."""
  # TODO(b/151975607): Clean this after robot interface migration.
  if hasattr(robot, "GetBaseOrientation"):
    return robot.GetBaseOrientation()
  else:
    return robot.base_orientation_quaternion
