# Lint as: python3
"""Defines the minitaur robot related constants and URDF specs."""

import collections
import math

import gin

MINITAUR_URDF_PATH = "quadruped/minitaur_rainbow_dash.urdf"

INIT_POSITION = (0, 0, 0.2)
INIT_RACK_POSITION = (0, 0, 1)
INIT_ORIENTATION_QUAT = (0, 0, 0, 1)
INIT_ORIENTATION_RPY = (0, 0, 0)

NUM_LEGS = 4

JOINT_NAMES = ("motor_front_leftL_joint", "motor_front_leftR_joint",
               "motor_back_leftL_joint", "motor_back_leftR_joint",
               "motor_front_rightL_joint", "motor_front_rightR_joint",
               "motor_back_rightL_joint", "motor_back_rightR_joint")

INIT_JOINT_ANGLES = collections.OrderedDict(
    zip(JOINT_NAMES, [math.pi / 2, math.pi / 2] * NUM_LEGS))

# Used to convert the robot SDK joint angles to URDF joint angles.
JOINT_DIRECTIONS = collections.OrderedDict(
    zip(JOINT_NAMES, (-1, -1, -1, -1, 1, 1, 1, 1)))

# Used to convert the robot SDK joint angles to URDF joint angles.
JOINT_OFFSETS = collections.OrderedDict(
    zip(JOINT_NAMES, (0, 0, 0, 0, 0, 0, 0, 0)))

LEG_ORDER = ["front_left", "back_left", "front_right", "back_right"]

END_EFFECTOR_NAMES = (
    "knee_front_rightR_joint",
    "knee_front_leftL_joint",
    "knee_back_rightR_joint",
    "knee_back_leftL_joint",
)

MOTOR_NAMES = JOINT_NAMES
MOTOR_GROUP = collections.OrderedDict((("body_motors", JOINT_NAMES),))

KNEE_CONSTRAINT_POINT_LONG = [0, 0.0045, 0.088]
KNEE_CONSTRAINT_POINT_SHORT = [0, 0.0045, 0.100]

# Add the gin constants to be used for gin binding in config.
gin.constant("minitaur_constants.MINITAUR_URDF_PATH", MINITAUR_URDF_PATH)
gin.constant("minitaur_constants.MINITAUR_INIT_POSITION", INIT_POSITION)
gin.constant("minitaur_constants.MINITAUR_INIT_ORIENTATION_QUAT",
             INIT_ORIENTATION_QUAT)
gin.constant("minitaur_constants.MINITAUR_INIT_ORIENTATION_RPY",
             INIT_ORIENTATION_RPY)
gin.constant("minitaur_constants.MINITAUR_INIT_JOINT_ANGLES", INIT_JOINT_ANGLES)
gin.constant("minitaur_constants.MINITAUR_JOINT_DIRECTIONS", JOINT_DIRECTIONS)
gin.constant("minitaur_constants.MINITAUR_JOINT_OFFSETS", JOINT_OFFSETS)
gin.constant("minitaur_constants.MINITAUR_MOTOR_NAMES", MOTOR_NAMES)
gin.constant("minitaur_constants.MINITAUR_END_EFFECTOR_NAMES",
             END_EFFECTOR_NAMES)
gin.constant("minitaur_constants.MINITAUR_MOTOR_GROUP", MOTOR_GROUP)
