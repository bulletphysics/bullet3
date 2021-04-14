# Lint as: python3
"""Interface that specifies tasks."""
# TODO(tingnan): Def. proper task interface - see TODO(b/154635313) in comments.

import abc
from typing import Sequence

import gym

from pybullet_envs.minitaur.envs_v2.sensors import sensor


class Task(metaclass=abc.ABCMeta):
  """Base class for tasks."""

  # TODO(b/154635313): Deprecate this method. Consolidate it into update().
  @abc.abstractmethod
  def reward(self, env: gym.Env) -> float:
    """Returns the reward for the current state of the environment."""

  @abc.abstractmethod
  def reset(self, env: gym.Env) -> None:
    """Resets the task."""

  @abc.abstractmethod
  def update(self, env: gym.Env) -> None:
    """Updates the internal state of the task."""

  # TODO(b/154635313): Deprecate this method. Consolidate it into update().
  @abc.abstractmethod
  def done(self, env: gym.Env) -> bool:
    """Determines whether the task is done."""

  @property
  def sensors(self) -> Sequence[sensor.Sensor]:
    """Returns sensors used by task."""
    return []
