"""Load the locomotion gym env using the gin config files."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import gin
from pybullet_envs.minitaur.envs_v2 import locomotion_gym_env
from pybullet_envs.minitaur.envs_v2 import multiagent_mobility_gym_env

ROBOT_FIELD_IN_CONFIG = 'robot_params'
TASK_FIELD_IN_CONFIG = 'task_params'
PMTG_FIELD_IN_CONFIG = 'pmtg_params'
_PMTG_GIN_QUERY = 'pmtg_wrapper_env.PmtgWrapperEnv.'


@gin.configurable
def load(wrapper_classes=None, multiagent=False, **kwargs):
  """load a pre-defined locomotion gym environment.

  The env specific settings should have been set in the gin files.

  Args:
    wrapper_classes: A list of wrapper_classes.
    multiagent: Whether to use multiagent environment.
    **kwargs: Keyword arguments to be passed to the environment constructor.

  Returns:
    env: The instance of the minitaur gym environment.
  """
  # Gin config are not always specified this way (e.g. namescoped config).
  # Only guery parameters when it is necessary.
  if any(
      k in (PMTG_FIELD_IN_CONFIG, TASK_FIELD_IN_CONFIG, ROBOT_FIELD_IN_CONFIG)
      for k in kwargs):
    with gin.unlock_config():
      if multiagent:
        #  Currently assume robots and tasks are identical
        robot_class = gin.query_parameter(
            'multiagent_mobility_gym_env.MultiagentMobilityGymEnv.robot_classes'
        )[0].selector
        task = gin.query_parameter(
            'multiagent_mobility_gym_env.MultiagentMobilityGymEnv.tasks'
        )[0].selector
      else:
        robot_class = gin.query_parameter(
            'locomotion_gym_env.LocomotionGymEnv.robot_class').selector
        task = gin.query_parameter(
            'locomotion_gym_env.LocomotionGymEnv.task').selector
      gin_prefix_dict = {
          PMTG_FIELD_IN_CONFIG: _PMTG_GIN_QUERY,
          TASK_FIELD_IN_CONFIG: task + '.',
          ROBOT_FIELD_IN_CONFIG: robot_class + '.',
      }
      for field_name, field_values in kwargs.items():
        if field_name in gin_prefix_dict:
          for var_name, value in field_values.items():
            gin.bind_parameter(gin_prefix_dict[field_name] + var_name, value)
        else:
          raise ValueError(
              'Environment argument type is not found in gin_prefix_dict.')
  if multiagent:
    env = multiagent_mobility_gym_env.MultiagentMobilityGymEnv()
  else:
    env = locomotion_gym_env.LocomotionGymEnv()
  if wrapper_classes is not None:
    # A little macro for the automatic list expansion
    if not isinstance(wrapper_classes, list):
      wrapper_classes = [wrapper_classes]

    # Wrap environments with user-provided wrappers
    # (e.g. TrajectoryGeneratorWrapperEnv)
    for wrapper_cls in wrapper_classes:
      env = wrapper_cls(env)
  return env
