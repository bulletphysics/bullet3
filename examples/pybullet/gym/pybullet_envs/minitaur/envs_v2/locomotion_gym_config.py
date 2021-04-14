"""A gin-config class for locomotion_gym_env.

This should be identical to locomotion_gym_config.proto.
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from typing import Sequence, Text

import attr
import gin


@gin.configurable
@attr.s
class SimulationParameters(object):
  """Parameters specific for the pyBullet simulation."""
  sim_time_step_s = attr.ib(type=float, default=0.002)
  num_action_repeat = attr.ib(type=int, default=5)
  enable_hard_reset = attr.ib(type=bool, default=False)
  enable_rendering = attr.ib(type=bool, default=False)
  enable_rendering_gui = attr.ib(type=bool, default=True)
  robot_on_rack = attr.ib(type=bool, default=False)
  camera_target = attr.ib(type=Sequence[float], default=None)
  camera_distance = attr.ib(type=float, default=1.0)
  camera_yaw = attr.ib(type=float, default=0)
  camera_pitch = attr.ib(type=float, default=-30)
  render_width = attr.ib(type=int, default=480)
  render_height = attr.ib(type=int, default=360)
  egl_rendering = attr.ib(type=bool, default=False)


@gin.configurable
@attr.s
class ScalarField(object):
  """A named scalar space with bounds."""
  # TODO(sehoonha) extension to vector fields.
  name = attr.ib(type=str)
  upper_bound = attr.ib(type=float)
  lower_bound = attr.ib(type=float)


@gin.configurable
@attr.s
class LocomotionGymConfig(object):
  """Grouped Config Parameters for LocomotionGym."""
  simulation_parameters = attr.ib(type=SimulationParameters)
  # TODO(sehoonha) implement attr validators for the list
  actions = attr.ib(type=list, default=None)  # pylint: disable=g-bare-generic
  log_path = attr.ib(type=Text, default=None)
  data_dir = attr.ib(
      type=Text,
      default='robotics/reinforcement_learning/minitaur/data/')
  profiling_path = attr.ib(type=Text, default=None)
  seed = attr.ib(type=int, default=None)
  ignored_sensor_list = attr.ib(type=Sequence[Text], default=())
