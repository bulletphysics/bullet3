"""Abstract base class for environment randomizer."""

import abc


class EnvRandomizerBase(object):
  """Abstract base class for environment randomizer.

  Randomizes physical parameters of the objects in the simulation and adds
  perturbations to the stepping of the simulation.
  """

  __metaclass__ = abc.ABCMeta

  @abc.abstractmethod
  def randomize_env(self, env):
    """Randomize the simulated_objects in the environment.

    Will be called at when env is reset. The physical parameters will be fixed
    for that episode and be randomized again in the next environment.reset().

    Args:
      env: The Minitaur gym environment to be randomized.
    """
    pass

  def randomize_step(self, env):
    """Randomize environment steps.

    Will be called at every environment step.

    It is NOT recommended to use this for force / torque disturbance because
    pybullet applyExternalForce/Torque only persist for single simulation step
    not the entire env step which can contain multiple simulation steps.

    Args:
      env: The Minitaur gym environment to be randomized.
    """
    pass

  def randomize_sub_step(self, env, sub_step_index, num_sub_steps):
    """Randomize simulation sub steps.

    Will be called at every simulation step. This is the correct place to add
    random forces/torques.

    Args:
      env: The Minitaur gym environment to be randomized.
      sub_step_index: Index of sub step, from 0 to N-1. N is the action repeat.
      num_sub_steps: Number of sub steps, equals to action repeat.
    """
    pass
