"""Utility functions for the Minitaur Raibert controller."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import math

UPPER_LEG_LEN = 0.112
LOWER_SHORT_LEG_LEN = 0.199
LOWER_LONG_LEG_LEN = 0.2315


def leg_pose_to_foot_position(leg_pose):
  """The forward kinematics."""
  l1 = UPPER_LEG_LEN
  l2 = LOWER_SHORT_LEG_LEN
  l3 = LOWER_LONG_LEG_LEN

  ext = leg_pose[1]
  alpha = math.asin(l1 * math.sin(ext) / l2)

  sw = -leg_pose[0]
  x = l3 * math.sin(alpha + sw) - l1 * math.sin(ext + sw)
  y = l3 * math.cos(alpha + sw) - l1 * math.cos(ext + sw)

  return (x, -y)


def foot_position_to_leg_pose(foot_position):
  """The inverse kinematics."""
  l1 = UPPER_LEG_LEN
  l2 = LOWER_SHORT_LEG_LEN
  l3 = LOWER_LONG_LEG_LEN

  x = foot_position[0]
  y = foot_position[1]

  assert y < 0
  hip_toe_sqr = x**2 + y**2
  cos_beta = (l1 * l1 + l3 * l3 - hip_toe_sqr) / (2 * l1 * l3)
  assert -1 <= cos_beta <= 1
  hip_ankle_sqr = l1 * l1 + l2 * l2 - 2 * l1 * l2 * cos_beta
  hip_ankle = math.sqrt(hip_ankle_sqr)
  cos_ext = -(l1 * l1 + hip_ankle_sqr - l2 * l2) / (2 * l1 * hip_ankle)
  ext = math.acos(cos_ext)

  hip_toe = math.sqrt(hip_toe_sqr)
  cos_theta = (hip_toe_sqr + hip_ankle_sqr -
               (l3 - l2)**2) / (2 * hip_ankle * hip_toe)

  assert cos_theta > 0
  theta = math.acos(cos_theta)
  sw = math.asin(x / hip_toe) - theta
  return (-sw, ext)


def extension_to_ankle_dist(extension):
  """Converts leg extension to ankle-hip distance in meters.

  The ankle is defined as the joint of the two lower links, which is different
  from the toe which is the tip of the longer lower limb.

  Args:
    extension: Float. the leg extension.

  Returns:
    Float. The hip to ankle distance in meters.

  """
  l1 = UPPER_LEG_LEN
  l2 = LOWER_SHORT_LEG_LEN
  alpha = math.asin(l1 / l2 * math.sin(extension))
  return l2 * math.cos(alpha) - l1 * math.cos(extension)


def ankle_dist_to_extension(dist):
  """Converts ankle-hip distance (meters) to extension."""
  l1 = UPPER_LEG_LEN
  l2 = LOWER_SHORT_LEG_LEN
  cos_extension = -(l1**2 + dist**2 - l2**2) / (2 * l1 * dist)
  return math.acos(cos_extension)
