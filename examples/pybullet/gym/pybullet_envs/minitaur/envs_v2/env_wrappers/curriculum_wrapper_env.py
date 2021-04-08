"""A wrapped LocomotionGymEnv with functions that change the world and task."""

import gin


@gin.configurable
class CurriculumWrapperEnv(object):
  """A wrapped LocomotionGymEnv with an evolving environment."""

  def __init__(self,
               gym_env,
               num_iter=0,
               curriculum_world_update=None,
               curriculum_task_update=None):
    """Initializes the wrapped env.

    Args:
      gym_env: An instance of a (potentially previously wrapped)
        LocomotionGymEnv.
      num_iter: The training iteration we are on.
      curriculum_world_update: A function that updates the environment based on
        the iteration. Takes in the environment as an argument.
      curriculum_task_update: A function that updates the task (eg.
        observations) based on the iteration. Takes in the environment as an
        argument.
    """
    self._gym_env = gym_env
    self._num_iter = num_iter
    self._curriculum_world_update = curriculum_world_update
    self._curriculum_task_update = curriculum_task_update

  def modify(self, step=0):
    self._num_iter = step

  def __getattr__(self, attr):
    return getattr(self._gym_env, attr)

  def reset(self, *args, **kwargs):
    """Reset and adjust the environment."""
    self._gym_env.reset(*args, **kwargs)
    if self._curriculum_world_update is not None:
      self._curriculum_world_update(self._gym_env, self._num_iter)
    if self._curriculum_task_update is not None:
      self._curriculum_task_update(self._gym_env, self._num_iter)
    return self._get_observation()

  # Used for testing.
  @property
  def num_iter(self):
    return self._num_iter
