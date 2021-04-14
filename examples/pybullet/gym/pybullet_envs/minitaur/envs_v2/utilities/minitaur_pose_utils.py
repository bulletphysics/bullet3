# coding=utf-8
# Copyright 2020 The Google Research Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Utility functions to calculate Minitaur's pose and motor angles."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import math
import attr
import numpy as np

NUM_MOTORS = 8
NUM_LEGS = 4
MOTOR_SIGNS = (1, 1, -1, -1)
# Constants for the function swing_extend_to_motor_angles
EPS = 0.1
# Range of motion for the legs (does not allow pointing towards the body).
LEG_SWING_LIMIT_LOW = -math.pi / 2 + EPS
LEG_SWING_LIMIT_HIGH = 3 * math.pi / 2 - EPS
# Range of gap between motors for feasibility.
MOTORS_GAP_LIMIT_HIGH = 2 * math.pi - EPS
MOTORS_GAP_LIMIT_LOW = EPS


@attr.s
class MinitaurPose(object):
  """Default pose of the Minitaur."""
  swing_angle_0 = attr.ib(type=float, default=0)
  swing_angle_1 = attr.ib(type=float, default=0)
  swing_angle_2 = attr.ib(type=float, default=0)
  swing_angle_3 = attr.ib(type=float, default=0)
  extension_angle_0 = attr.ib(type=float, default=0)
  extension_angle_1 = attr.ib(type=float, default=0)
  extension_angle_2 = attr.ib(type=float, default=0)
  extension_angle_3 = attr.ib(type=float, default=0)


def motor_angles_to_leg_pose(motor_angles):
  """Convert motor angles to the leg pose.

  A single leg pose is a tuple (swing, extension). The definition can be find
  in:
    Sim-to-Real: Learning Agile Locomotion For Quadruped Robot

  Args:
    motor_angles: A numpy array. Contains all eight motor angles for Minitaur.

  Returns:
    A numpy array. Contains the leg pose for all four legs: [swing_0, swing_1,
    swing_2, swing_3, extension_0, extension_1, extension_2, extension_3]

  """
  motor_angles = np.array(motor_angles)

  swings = 0.5 * np.multiply(
      np.array(MOTOR_SIGNS), (motor_angles[1::2] - motor_angles[::2]))
  extensions = 0.5 * (motor_angles[::2] + motor_angles[1::2])

  return np.concatenate((swings, extensions), axis=None)


def leg_pose_to_motor_angles(leg_pose):
  """Converts the leg pose to the motor angles.

  Args:
    leg_pose: A numpy array. Contains the leg pose for all four legs: [swing_0,
      swing_1, swing_2, swing_3, extension_0, extension_1, extension_2,
      extension_3]

  Returns:
    A numpy array. All eight motor angles.
  """
  leg_pose = np.array(leg_pose)

  # swings multiplied with the sign array.
  signed_swings = np.multiply(np.array(MOTOR_SIGNS), leg_pose[0:NUM_LEGS])
  extensions = leg_pose[NUM_LEGS:]

  motor_angles = np.zeros(NUM_MOTORS)
  motor_angles[1::2] = signed_swings + extensions
  motor_angles[::2] = extensions - signed_swings
  return motor_angles


# This method also does the same conversion, but 0 swing and 0 extension maps
# to a neutral standing still motor positions with motors at + or - pi. It also
# contains a safety layer so that the legs don't swing or extend too much to hit
# the body of the robot.
def leg_pose_to_motor_angles_with_half_pi_offset_and_safety(leg_pose):
  """Converts the swing extension poses to the motor angles with safety limits.

  Args:
    leg_pose: A numpy array. Contains the leg pose for all four legs: [swing_0,
      extension_0, swing_1, extension_1, swing_2, extension_2, swing_3,
      extension_3]

  Returns:
    A numpy array. All eight motor angles.
  """

  motor_angles = []
  for idx in range(4):
    swing = leg_pose[idx * 2]
    extend = leg_pose[idx * 2 + 1]
    motor_angles.extend(swing_extend_to_motor_angles(idx, swing, extend))
  return motor_angles


def swing_extend_to_motor_angles(leg_id, swing, extension, noise_stdev=0):
  """Swing - extension based leg model for minitaur.

  Swing extension leg model calculates motor positions using 2 separate motions:
  swing and extension. Swing rotates the whole leg by rotating both motors
  equally towards same direction. Extension increases or decreases the length
  of the leg by turning both motors equally in opposite direction.

  This method also does the same conversion as leg_pose_to_motor_angles, but 0
  swing and 0 extension maps to a neutral standing still motor positions with
  motors at + or - pi.
  Args:
      leg_id: The id of the leg that the conversion is made for (0, 1, 2, 3).
      swing: Swing degree for the leg (in radians). 0 means perpendicular to the
        body).
      extension: Extension level (length) of the leg, limited to [-1, 1].
      noise_stdev: Standard deviation of the introduced noise at the motor
        position level. Noise is turned off by default.

  Returns:
    motor0: Position for the first motor for that leg.
    motor1: Position for the second motor for that leg.
  Raises:
    ValueError: In case calculated positions are outside the allowed boundaries.
  """
  # Check if the leg_id is in valid range
  if not 0 <= leg_id <= 3:
    raise ValueError('leg {} does not exist for a quadruped.'.format(leg_id))

  # Front legs can not swing too much towards the body.
  if leg_id % 2 == 0:
    swing = np.clip(swing, LEG_SWING_LIMIT_LOW, LEG_SWING_LIMIT_HIGH)
  # Back legs can not swing too much towards the body (opposite direction).
  else:
    swing = np.clip(swing, -LEG_SWING_LIMIT_HIGH, -LEG_SWING_LIMIT_LOW)

  # Check if the motors are too close or too far away to make it impossible
  # for the physical robot.
  gap = math.pi - 2 * extension
  if gap < MOTORS_GAP_LIMIT_LOW or gap > MOTORS_GAP_LIMIT_HIGH:
    top_extension = (math.pi - MOTORS_GAP_LIMIT_LOW) / 2.0
    least_extension = (math.pi - MOTORS_GAP_LIMIT_HIGH) / 2.0
    extension = np.clip(extension, least_extension, top_extension)

  # Initialization to neutral standing position where both motors point to
  # opposite directions
  motor0 = math.pi / 2
  motor1 = math.pi / 2
  # Rotational move
  if leg_id in (0, 1):
    motor0 += swing
    motor1 -= swing
  elif leg_id in (2, 3):
    motor0 -= swing
    motor1 += swing
  # Extension
  motor0 += extension
  motor1 += extension

  # Add noise if requested.
  if noise_stdev > 0:
    motor0 += np.random.normal(0, noise_stdev)
    motor1 += np.random.normal(0, noise_stdev)

  return motor0, motor1
