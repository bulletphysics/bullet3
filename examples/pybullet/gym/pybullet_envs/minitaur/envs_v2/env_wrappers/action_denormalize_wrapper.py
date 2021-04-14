"""Denormalize the action from [-1, 1] to the env.action_space."""

import gin
import gym
import numpy as np


def _denomralize(env, action):
  action = np.array(action)
  low = np.array(env.action_space.low)
  high = np.array(env.action_space.high)
  return (high - low) / 2.0 * action + (high + low) / 2.0


@gin.configurable
class ActionDenormalizeWrapper(object):
  """An env wrapper that denormalize the action from [-1, 1] to the bounds."""

  def __init__(self, gym_env):
    """Initializes the wrapper."""
    self._gym_env = gym_env
    self.action_space = gym.spaces.Box(
        low=-1.0,
        high=1.0,
        shape=self._gym_env.action_space.low.shape,
        dtype=np.float32)

  def __getattr__(self, attr):
    return getattr(self._gym_env, attr)

  def step(self, action):
    """Steps the wrapped environment.

    Args:
      action: Numpy array between [-1.0, 1.0].

    Returns:
      The tuple containing the observation, the reward, and the epsiode
        end indicator.
    """
    return self._gym_env.step(_denomralize(self._gym_env, action))
