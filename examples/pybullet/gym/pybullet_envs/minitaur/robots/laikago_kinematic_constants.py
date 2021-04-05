# Lint as: python3
"""Defines the LaikagoKinematic robot related constants and URDF specs."""

import collections
import gin

LAIKAGO_KINEMATIC_URDF_PATH = "robotics/reinforcement_learning/minitaur/robots/data/urdf/laikago/laikago_toes_zup_kinematic.urdf"
INIT_POSITION = (0, 0, 0.5)
INIT_ORIENTATION_QUAT = (0, 0, 0, 1)
INIT_ORIENTATION_RPY = (0, 0, 0)

NUM_LEGS = 4
# TODO(b/153405332): Use link name instead of joint name to identify the
# base link.
BASE_NAMES = ("kinematic_drive_joint_th",)

JOINT_NAMES = (
    "kinematic_drive_joint_x",
    "kinematic_drive_joint_y",
    "kinematic_drive_joint_th",
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

# A default joint pose where the arm is tucked near the base, and head looking
# forward.
INIT_ABDUCTION_ANGLE = 0
INIT_HIP_ANGLE = 0.67
INIT_KNEE_ANGLE = -1.25

# Note this matches the Laikago SDK/control convention, but is different from
# URDF's internal joint angles which needs to be computed using the joint
# offsets and directions. The conversion formula is (sdk_joint_angle + offset) *
# joint direction.
INIT_JOINT_ANGLES = collections.OrderedDict(
    zip(JOINT_NAMES, (0, 0, 0) +
        (INIT_ABDUCTION_ANGLE, INIT_HIP_ANGLE, INIT_KNEE_ANGLE) * NUM_LEGS))

# Used to convert the robot SDK joint angles to URDF joint angles.
JOINT_DIRECTIONS = collections.OrderedDict(
    zip(JOINT_NAMES, (1, 1, 1, -1, 1, 1, 1, 1, 1, -1, 1, 1, 1, 1, 1)))

HIP_JOINT_OFFSET = 0.0
UPPER_LEG_JOINT_OFFSET = -0.6
KNEE_JOINT_OFFSET = 0.66

# Used to convert the robot SDK joint angles to URDF joint angles.
JOINT_OFFSETS = collections.OrderedDict(
    zip(JOINT_NAMES, [0, 0, 0] +
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
MOTOR_GROUP = collections.OrderedDict((("body_motors", JOINT_NAMES[3:]),))

# Add the gin constants to be used for gin binding in config.
gin.constant("laikago_kinematic_constants.LAIKAGO_KINEMATIC_URDF_PATH",
             LAIKAGO_KINEMATIC_URDF_PATH)
gin.constant("laikago_kinematic_constants.INIT_POSITION", INIT_POSITION)
gin.constant("laikago_kinematic_constants.INIT_ORIENTATION_QUAT",
             INIT_ORIENTATION_QUAT)
gin.constant("laikago_kinematic_constants.INIT_ORIENTATION_RPY",
             INIT_ORIENTATION_RPY)
gin.constant("laikago_kinematic_constants.BASE_NAMES", BASE_NAMES)
gin.constant("laikago_kinematic_constants.INIT_JOINT_ANGLES", INIT_JOINT_ANGLES)
gin.constant("laikago_kinematic_constants.JOINT_DIRECTIONS", JOINT_DIRECTIONS)
gin.constant("laikago_kinematic_constants.JOINT_OFFSETS", JOINT_OFFSETS)
gin.constant("laikago_kinematic_constants.MOTOR_NAMES", MOTOR_NAMES)
gin.constant("laikago_kinematic_constants.END_EFFECTOR_NAMES",
             END_EFFECTOR_NAMES)
gin.constant("laikago_kinematic_constants.MOTOR_GROUP", MOTOR_GROUP)
