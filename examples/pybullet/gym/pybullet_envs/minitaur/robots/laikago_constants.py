# Lint as: python3
"""Defines the laikago robot related constants and URDF specs."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import collections
import gin

URDF_PATH = "laikago/laikago_toes_zup.urdf"

NUM_MOTORS = 12
NUM_LEGS = 4
MOTORS_PER_LEG = 3

INIT_RACK_POSITION = [0, 0, 1]
INIT_POSITION = [0, 0, 0.48]

# Will be default to (0, 0, 0, 1) once the new laikago_toes_zup.urdf checked in.
INIT_ORIENTATION = [0, 0, 0, 1]

# Can be different from the motors, although for laikago they are the same list.
JOINT_NAMES = (
    # front right leg
    "FR_hip_motor_2_chassis_joint",
    "FR_upper_leg_2_hip_motor_joint",
    "FR_lower_leg_2_upper_leg_joint",
    # front left leg
    "FL_hip_motor_2_chassis_joint",
    "FL_upper_leg_2_hip_motor_joint",
    "FL_lower_leg_2_upper_leg_joint",
    # rear right leg
    "RR_hip_motor_2_chassis_joint",
    "RR_upper_leg_2_hip_motor_joint",
    "RR_lower_leg_2_upper_leg_joint",
    # rear left leg
    "RL_hip_motor_2_chassis_joint",
    "RL_upper_leg_2_hip_motor_joint",
    "RL_lower_leg_2_upper_leg_joint",
)

INIT_ABDUCTION_ANGLE = 0
INIT_HIP_ANGLE = 0.67
INIT_KNEE_ANGLE = -1.25

# Note this matches the Laikago SDK/control convention, but is different from
# URDF's internal joint angles which needs to be computed using the joint
# offsets and directions. The conversion formula is (sdk_joint_angle + offset) *
# joint direction.
INIT_JOINT_ANGLES = collections.OrderedDict(
    zip(JOINT_NAMES,
        (INIT_ABDUCTION_ANGLE, INIT_HIP_ANGLE, INIT_KNEE_ANGLE) * NUM_LEGS))

# Used to convert the robot SDK joint angles to URDF joint angles.
JOINT_DIRECTIONS = collections.OrderedDict(
    zip(JOINT_NAMES, (-1, 1, 1, 1, 1, 1, -1, 1, 1, 1, 1, 1)))

HIP_JOINT_OFFSET = 0.0
UPPER_LEG_JOINT_OFFSET = -0.6
KNEE_JOINT_OFFSET = 0.66

# Used to convert the robot SDK joint angles to URDF joint angles.
JOINT_OFFSETS = collections.OrderedDict(
    zip(JOINT_NAMES,
        [HIP_JOINT_OFFSET, UPPER_LEG_JOINT_OFFSET, KNEE_JOINT_OFFSET] *
        NUM_LEGS))

LEG_NAMES = (
    "front_right",
    "front_left",
    "rear_right",
    "rear_left",
)

LEG_ORDER = (
    "front_right",
    "front_left",
    "back_right",
    "back_left",
)

END_EFFECTOR_NAMES = (
    "jtoeFR",
    "jtoeFL",
    "jtoeRR",
    "jtoeRL",
)

MOTOR_NAMES = JOINT_NAMES
MOTOR_GROUP = collections.OrderedDict((
    (LEG_NAMES[0], JOINT_NAMES[0:3]),
    (LEG_NAMES[1], JOINT_NAMES[3:6]),
    (LEG_NAMES[2], JOINT_NAMES[6:9]),
    (LEG_NAMES[3], JOINT_NAMES[9:12]),
))

# Regulates the joint angle change when in position control mode.
MAX_MOTOR_ANGLE_CHANGE_PER_STEP = 0.12

# The hip joint location in the CoM frame.
HIP_POSITIONS = collections.OrderedDict((
    (LEG_NAMES[0], (0.21, -0.1157, 0)),
    (LEG_NAMES[1], (0.21, 0.1157, 0)),
    (LEG_NAMES[2], (-0.21, -0.1157, 0)),
    (LEG_NAMES[3], (-0.21, 0.1157, 0)),
))

# Add the gin constants to be used for gin binding in config. Append "LAIKAGO_"
# for unique binding names.
gin.constant("laikago_constants.LAIKAGO_NUM_MOTORS", NUM_MOTORS)
gin.constant("laikago_constants.LAIKAGO_URDF_PATH", URDF_PATH)
gin.constant("laikago_constants.LAIKAGO_INIT_POSITION", INIT_POSITION)
gin.constant("laikago_constants.LAIKAGO_INIT_ORIENTATION", INIT_ORIENTATION)
gin.constant("laikago_constants.LAIKAGO_INIT_JOINT_ANGLES", INIT_JOINT_ANGLES)
gin.constant("laikago_constants.LAIKAGO_JOINT_DIRECTIONS", JOINT_DIRECTIONS)
gin.constant("laikago_constants.LAIKAGO_JOINT_OFFSETS", JOINT_OFFSETS)
gin.constant("laikago_constants.LAIKAGO_MOTOR_NAMES", MOTOR_NAMES)
gin.constant("laikago_constants.LAIKAGO_END_EFFECTOR_NAMES", END_EFFECTOR_NAMES)
gin.constant("laikago_constants.LAIKAGO_MOTOR_GROUP", MOTOR_GROUP)
