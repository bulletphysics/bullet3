"""Utility functions to calculate Mini Cheetah's pose and motor angles."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import attr

MINI_CHEETAH_DEFAULT_ABDUCTION_ANGLE = 0
MINI_CHEETAH_DEFAULT_HIP_ANGLE = -1.2
MINI_CHEETAH_DEFAULT_KNEE_ANGLE = 2.1


@attr.s
class MiniCheetahPose(object):
  """Default pose of the Laikago.

    Leg order:
    0 -> Front Right.
    1 -> Front Left.
    2 -> Rear Right.
    3 -> Rear Left.
  """
  abduction_angle_0 = attr.ib(type=float, default=0)
  hip_angle_0 = attr.ib(type=float, default=0)
  knee_angle_0 = attr.ib(type=float, default=0)
  abduction_angle_1 = attr.ib(type=float, default=0)
  hip_angle_1 = attr.ib(type=float, default=0)
  knee_angle_1 = attr.ib(type=float, default=0)
  abduction_angle_2 = attr.ib(type=float, default=0)
  hip_angle_2 = attr.ib(type=float, default=0)
  knee_angle_2 = attr.ib(type=float, default=0)
  abduction_angle_3 = attr.ib(type=float, default=0)
  hip_angle_3 = attr.ib(type=float, default=0)
  knee_angle_3 = attr.ib(type=float, default=0)
