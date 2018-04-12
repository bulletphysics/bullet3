"""This file implements the gym environment of minitaur alternating legs.

"""

import os,  inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)

import collections
import math
from gym import spaces
import numpy as np
from pybullet_envs.minitaur.envs import minitaur_gym_env

INIT_EXTENSION_POS = 2.0
INIT_SWING_POS = 0.0
NUM_LEGS = 4
NUM_MOTORS = 2 * NUM_LEGS

MinitaurPose = collections.namedtuple(
    "MinitaurPose",
    "swing_angle_1, swing_angle_2, swing_angle_3, swing_angle_4, "
    "extension_angle_1, extension_angle_2, extension_angle_3, "
    "extension_angle_4")


class MinitaurReactiveEnv(minitaur_gym_env.MinitaurGymEnv):
  """The gym environment for the minitaur.

  It simulates the locomotion of a minitaur, a quadruped robot. The state space
  include the angles, velocities and torques for all the motors and the action
  space is the desired motor angle for each motor. The reward function is based
  on how far the minitaur walks in 1000 steps and penalizes the energy
  expenditure.

  """
  metadata = {
      "render.modes": ["human", "rgb_array"],
      "video.frames_per_second": 166
  }

  def __init__(self,
               urdf_version=None,
               energy_weight=0.005,
               control_time_step=0.006,
               action_repeat=6,
               control_latency=0.02,
               pd_latency=0.003,
               on_rack=False,
               motor_kp=1.0,
               motor_kd=0.015,
               remove_default_joint_damping=True,
               render=False,
               num_steps_to_log=1000,
               accurate_motor_model_enabled=True,
               use_angle_in_observation=True,
               hard_reset=False,
               env_randomizer=None,
               log_path=None):
    """Initialize the minitaur trotting gym environment.

    Args:
      urdf_version: [DEFAULT_URDF_VERSION, DERPY_V0_URDF_VERSION] are allowable
        versions. If None, DEFAULT_URDF_VERSION is used. Refer to
        minitaur_gym_env for more details.
      energy_weight: The weight of the energy term in the reward function. Refer
        to minitaur_gym_env for more details.
      control_time_step: The time step between two successive control signals.
      action_repeat: The number of simulation steps that an action is repeated.
      control_latency: The latency between get_observation() and the actual
        observation. See minituar.py for more details.
      pd_latency: The latency used to get motor angles/velocities used to
        compute PD controllers. See minitaur.py for more details.
      on_rack: Whether to place the minitaur on rack. This is only used to debug
        the walking gait. In this mode, the minitaur"s base is hung midair so
        that its walking gait is clearer to visualize.
      motor_kp: The P gain of the motor.
      motor_kd: The D gain of the motor.
      remove_default_joint_damping: Whether to remove the default joint damping.
      render: Whether to render the simulation.
      num_steps_to_log: The max number of control steps in one episode. If the
        number of steps is over num_steps_to_log, the environment will still
        be running, but only first num_steps_to_log will be recorded in logging.
      accurate_motor_model_enabled: Whether to use the accurate motor model from
        system identification. Refer to minitaur_gym_env for more details.
      use_angle_in_observation: Whether to include motor angles in observation.
      hard_reset: Whether hard reset (swipe out everything and reload) the
        simulation. If it is false, the minitaur is set to the default pose
        and moved to the origin.
      env_randomizer: An instance (or a list) of EnvRanzomier(s) that can
        randomize the environment during when env.reset() is called and add
        perturbations when env._step() is called.
      log_path: The path to write out logs. For the details of logging, refer to
        minitaur_logging.proto.
    """
    self._use_angle_in_observation = use_angle_in_observation

    super(MinitaurReactiveEnv, self).__init__(
        urdf_version=urdf_version,
        energy_weight=energy_weight,
        accurate_motor_model_enabled=accurate_motor_model_enabled,
        motor_overheat_protection=True,
        motor_kp=motor_kp,
        motor_kd=motor_kd,
        remove_default_joint_damping=remove_default_joint_damping,
        control_latency=control_latency,
        pd_latency=pd_latency,
        on_rack=on_rack,
        render=render,
        hard_reset=hard_reset,
        num_steps_to_log=num_steps_to_log,
        env_randomizer=env_randomizer,
        log_path=log_path,
        control_time_step=control_time_step,
        action_repeat=action_repeat)

    action_dim = 8
    action_low = np.array([-0.5] * action_dim)
    action_high = -action_low
    self.action_space = spaces.Box(action_low, action_high)
    self._cam_dist = 1.0
    self._cam_yaw = 30
    self._cam_pitch = -30

  def _reset(self):
    # TODO(b/73666007): Use composition instead of inheritance.
    # (http://go/design-for-testability-no-inheritance).
    init_pose = MinitaurPose(
        swing_angle_1=INIT_SWING_POS,
        swing_angle_2=INIT_SWING_POS,
        swing_angle_3=INIT_SWING_POS,
        swing_angle_4=INIT_SWING_POS,
        extension_angle_1=INIT_EXTENSION_POS,
        extension_angle_2=INIT_EXTENSION_POS,
        extension_angle_3=INIT_EXTENSION_POS,
        extension_angle_4=INIT_EXTENSION_POS)
    # TODO(b/73734502): Refactor input of _convert_from_leg_model to namedtuple.
    initial_motor_angles = self._convert_from_leg_model(list(init_pose))
    super(MinitaurReactiveEnv, self)._reset(
        initial_motor_angles=initial_motor_angles, reset_duration=0.5)
    return self._get_observation()

  def _convert_from_leg_model(self, leg_pose):
    motor_pose = np.zeros(NUM_MOTORS)
    for i in range(NUM_LEGS):
      motor_pose[int(2 * i)] = leg_pose[NUM_LEGS + i] - (-1)**int(i / 2) * leg_pose[i]
      motor_pose[int(2 * i + 1)] = (
          leg_pose[NUM_LEGS + i] + (-1)**int(i / 2) * leg_pose[i])
    return motor_pose

  def _signal(self, t):
    initial_pose = np.array([
        INIT_SWING_POS, INIT_SWING_POS, INIT_SWING_POS, INIT_SWING_POS,
        INIT_EXTENSION_POS, INIT_EXTENSION_POS, INIT_EXTENSION_POS,
        INIT_EXTENSION_POS
    ])
    return initial_pose

  def _transform_action_to_motor_command(self, action):
    # Add the reference trajectory (i.e. the trotting signal).
    action += self._signal(self.minitaur.GetTimeSinceReset())
    return self._convert_from_leg_model(action)

  def is_fallen(self):
    """Decides whether the minitaur is in a fallen state.

    If the roll or the pitch of the base is greater than 0.3 radians, the
    minitaur is considered fallen.

    Returns:
      Boolean value that indicates whether the minitaur has fallen.
    """
    roll, pitch, _ = self.minitaur.GetTrueBaseRollPitchYaw()
    return math.fabs(roll) > 0.3 or math.fabs(pitch) > 0.3

  def _get_true_observation(self):
    """Get the true observations of this environment.

    It includes the roll, the pitch, the roll dot and the pitch dot of the base.
    If _use_angle_in_observation is true, eight motor angles are added into the
    observation.

    Returns:
      The observation list, which is a numpy array of floating-point values.
    """
    roll, pitch, _ = self.minitaur.GetTrueBaseRollPitchYaw()
    roll_rate, pitch_rate, _ = self.minitaur.GetTrueBaseRollPitchYawRate()
    observation = [roll, pitch, roll_rate, pitch_rate]
    if self._use_angle_in_observation:
      observation.extend(self.minitaur.GetMotorAngles().tolist())
    self._true_observation = np.array(observation)
    return self._true_observation

  def _get_observation(self):
    roll, pitch, _ = self.minitaur.GetBaseRollPitchYaw()
    roll_rate, pitch_rate, _ = self.minitaur.GetBaseRollPitchYawRate()
    observation = [roll, pitch, roll_rate, pitch_rate]
    if self._use_angle_in_observation:
      observation.extend(self.minitaur.GetMotorAngles().tolist())
    self._observation = np.array(observation)
    return self._observation

  def _get_observation_upper_bound(self):
    """Get the upper bound of the observation.

    Returns:
      The upper bound of an observation. See _get_true_observation() for the
      details of each element of an observation.
    """
    upper_bound_roll = 2 * math.pi
    upper_bound_pitch = 2 * math.pi
    upper_bound_roll_dot = 2 * math.pi / self._time_step
    upper_bound_pitch_dot = 2 * math.pi / self._time_step
    upper_bound_motor_angle = 2 * math.pi
    upper_bound = [
        upper_bound_roll, upper_bound_pitch, upper_bound_roll_dot,
        upper_bound_pitch_dot
    ]

    if self._use_angle_in_observation:
      upper_bound.extend([upper_bound_motor_angle] * NUM_MOTORS)
    return np.array(upper_bound)

  def _get_observation_lower_bound(self):
    lower_bound = -self._get_observation_upper_bound()
    return lower_bound
