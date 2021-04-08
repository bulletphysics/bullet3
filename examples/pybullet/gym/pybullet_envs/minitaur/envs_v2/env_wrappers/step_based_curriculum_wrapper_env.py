"""A wrapped LocomotionGymEnv with functions that change the world and task."""

import numbers
from typing import Optional, Sequence, Text, Tuple, Union

import numpy as np

import gin


@gin.configurable
class CurriculumParameter(object):
  """Definition of a env parameter tuned by a curriculum."""

  def __init__(self,
               name: Text,
               init_val: numbers.Real,
               bounds: Tuple[numbers.Real, numbers.Real]):
    """A parameter to tune throughout the curriculum.

    Args:
      name: The name of the curriculum parameter. This must be the name of
        attribute of the scene class.
      init_val: The value to use at the start of the curriculum.
      bounds: A tuple of [lower_bound, upper_bound] defining the minimum and
        maximum values of the parameter.
    """
    self.name = name

    if not isinstance(bounds[0], type(bounds[1])):
      raise ValueError("All elements in [bounds] must be of the same type.")

    if (init_val < min(bounds)) or (init_val > max(bounds)):
      raise ValueError("Initial parameter value must lie in range defined"
                       " by [bounds].")

    if not isinstance(init_val, type(bounds[0])):
      raise ValueError("[init_val] type must match the type of the elements"
                       " in [bounds].")

    self.init_val = init_val
    self.bounds = bounds
    self.dtype = type(init_val)


@gin.configurable
class RandomSamplingCurriculumParameter(CurriculumParameter):
  """Env parameter whose value is sampled randomly without curriculum."""

  def __init__(self, name: Text, bounds: Tuple[numbers.Real, numbers.Real]):
    """A parameter whose value to sample randomly.

    Args:
      name: The name of the parameter.
      bounds: A tuple of [lower_bound, upper_bound] defining the minimum and
        maximum values of the parameter.
    """
    super(RandomSamplingCurriculumParameter, self).__init__(
        name=name, init_val=bounds[0], bounds=bounds)

  def sample(
      self, step: numbers.Real,
      curriculum_steps: Optional[numbers.Real] = None) -> Union[int, float]:
    del step, curriculum_steps
    sampled_val = np.random.uniform(*self.bounds)

    if not isinstance(self.bounds[0], float):
      sampled_val = int(round(sampled_val))

    return sampled_val

  def __call__(
      self, step: numbers.Real,
      curriculum_steps: Optional[numbers.Real] = None) -> Union[int, float]:
    return self.sample(step, curriculum_steps)


@gin.configurable
class LinearStepBasedCurriculumParameter(CurriculumParameter):
  """Definition of a env parameter tuned by a linear time-based curriculum."""

  def __init__(self,
               name: Text,
               init_val: numbers.Real,
               bounds: Tuple[numbers.Real, numbers.Real],
               curriculum_steps: Optional[numbers.Real] = None):
    """A parameter to tune throughout the curriculum.

    Args:
      name: The name of the curriculum parameter. This must be the name of
        attribute of the scene class.
      init_val: The value to use at the start of the curriculum.
      bounds: A tuple of [lower_bound, upper_bound] defining the minimum and
        maximum values of the parameter.
      curriculum_steps: Integer defining the number of steps to take when
        varying the curriculum parameter value from the init_val to either
        bound. If None is specified, then the curriculum must provide the
        curriculum_steps when sampling.
    """
    super(LinearStepBasedCurriculumParameter, self).__init__(
        name=name, init_val=init_val, bounds=bounds)
    self.curriculum_steps = curriculum_steps

  def get_bounds_at_step(
      self, step: numbers.Real,
      curriculum_steps: Optional[numbers.Real] = None
    ) -> Tuple[Union[int, float], Union[int, float]]:
    """Compute the bounds of the parameter at the current step.

    Args:
      step: An integer defining the current timestep.
      curriculum_steps: Optional curriculum steps. Must be passed if not passed
        during initialization.
    Returns:
      A tuple containing the lower bound and upper bound at the current step.
    """

    if not self.curriculum_steps and not curriculum_steps:
      raise ValueError("curriculum_steps not defined. Must be passed upon"
                       " initialization or must specified by curriculum"
                       " wrapper env on method call.")

    if self.curriculum_steps:
      curriculum_steps = self.curriculum_steps

    prog = min(float(step) / curriculum_steps, 1.0)
    curr_lower_bound = self.init_val - prog * (self.init_val - self.bounds[0])
    curr_upper_bound = self.init_val + prog * (self.bounds[1] - self.init_val)

    if not isinstance(self.bounds[0], float):
      curr_lower_bound = int(round(curr_lower_bound))
      curr_upper_bound = int(round(curr_upper_bound))

    return (curr_lower_bound, curr_upper_bound)

  def sample(
      self, step: numbers.Real,
      curriculum_steps: Optional[numbers.Real] = None) -> Union[int, float]:
    sampled_val = np.random.uniform(*self.get_bounds_at_step(
        step, curriculum_steps))

    if not isinstance(self.bounds[0], float):
      sampled_val = int(round(sampled_val))

    return sampled_val

  def __call__(
      self, step: numbers.Real,
      curriculum_steps: Optional[numbers.Real] = None) -> Union[int, float]:
    return self.sample(step, curriculum_steps)


@gin.configurable
class Task(object):
  """Defines a single task in the environment and its corresponding params."""

  def __init__(self, name: Text,
               curriculum_parameters: Sequence[CurriculumParameter]):
    """Initialize the task.

    Args:
      name: The name of the task.
      curriculum_parameters: A list of CurriculumParameter instances which
        define the parameters that this task makes use of.
    """
    self.name = name
    self.curriculum_parameters = curriculum_parameters


@gin.configurable
class StepBasedCurriculumWrapperEnv(object):
  """A wrapper to tune the scene parameters linearly with the steps taken."""

  def __init__(self, env, tasks: Sequence[Task],
               default_curriculum_steps: Optional[numbers.Real] = None,
               reset_total_step_count_val: numbers.Real = -1,
               steps_before_curriculum_start: numbers.Real = 0):
    """Initializes the linear curriculum wrapper env.

    Args:
      env: An instance of a (potentially previously wrapped) LocomotionGymEnv.
      tasks: Various tasks to shuffle through throughout the curriculum.
      default_curriculum_steps: Optional default value for curriculum steps.
      reset_total_step_count_val: Step at which to reset the total step count.
        The internal total_step_count is reset to 0 once this value is reached.
      steps_before_curriculum_start: Steps to take in environment before the
        curriculum begins.
    """
    self._gym_env = env
    self._tasks = tasks
    self._default_curriculum_steps = default_curriculum_steps
    self._reset_total_step_count_val = reset_total_step_count_val
    self._steps_before_curriculum_start = steps_before_curriculum_start

    # Total number of environment steps.
    self._total_step_count = 0

  def __getattr__(self, attr):
    return getattr(self._gym_env, attr)

  def set_scene_params(self):
    # Choose a task at random.
    curr_task = np.random.choice(self._tasks)

    # Cycle through the task's parameters and set them in the scene.
    self._gym_env.scene.reset_scene_params()
    self._gym_env.scene.scene_type = curr_task.name
    for curriculum_parameter in curr_task.curriculum_parameters:
      setattr(
          self._gym_env.scene, curriculum_parameter.name,
          curriculum_parameter(
              self._total_step_count - self._steps_before_curriculum_start,
              self._default_curriculum_steps))

  def reset(self, *args, **kwargs):
    """Reset and adjust the environment."""

    # Update the total step count.
    self._total_step_count += self._gym_env.env_step_counter
    if self._reset_total_step_count_val >= 0:
      if self._total_step_count >= self._reset_total_step_count_val:
        self._total_step_count = 0

    if self._total_step_count < self._steps_before_curriculum_start:
      return self._get_observation()

    self.set_scene_params()
    self._gym_env.reset(*args, **kwargs)

    return self._get_observation()

