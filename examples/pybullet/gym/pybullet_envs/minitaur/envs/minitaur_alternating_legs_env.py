"""This file implements the gym environment of minitaur alternating legs.

"""
import math

import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

from gym import spaces
import numpy as np
from pybullet_envs.minitaur.envs import minitaur_gym_env

INIT_EXTENSION_POS = 2.6
INIT_SWING_POS = 0.0
DESIRED_PITCH = 0
NUM_LEGS = 4
NUM_MOTORS = 2 * NUM_LEGS
STEP_PERIOD = 1.0 / 3.0  # Three steps per second.
STEP_AMPLITUDE = 0.75


class MinitaurAlternatingLegsEnv(minitaur_gym_env.MinitaurGymEnv):
  """The gym environment for the minitaur.

  It simulates the locomotion of a minitaur, a quadruped robot. The state space
  include the angles, velocities and torques for all the motors and the action
  space is the desired motor angle for each motor. The reward function is based
  on how far the minitaur walks in 1000 steps and penalizes the energy
  expenditure.

  """
  metadata = {"render.modes": ["human", "rgb_array"], "video.frames_per_second": 66}

  def __init__(self,
               urdf_version=None,
               control_time_step=0.006,
               action_repeat=6,
               control_latency=0,
               pd_latency=0,
               on_rack=False,
               motor_kp=1.0,
               motor_kd=0.02,
               remove_default_joint_damping=False,
               render=False,
               num_steps_to_log=1000,
               env_randomizer=None,
               log_path=None):
    """Initialize the minitaur alternating legs gym environment.

    Args:
      urdf_version: [DEFAULT_URDF_VERSION, DERPY_V0_URDF_VERSION] are allowable
        versions. If None, DEFAULT_URDF_VERSION is used. Refer to
        minitaur_gym_env for more details.
      control_time_step: The time step between two successive control signals.
      action_repeat: The number of simulation steps that an action is repeated.
      control_latency: The latency between get_observation() and the actual
        observation. See minituar.py for more details.
      pd_latency: The latency used to get motor angles/velocities used to
        compute PD controllers. See minitaur.py for more details.
      on_rack: Whether to place the minitaur on rack. This is only used to debug
        the walking gait. In this mode, the minitaur's base is hung midair so
        that its walking gait is clearer to visualize.
      motor_kp: The P gain of the motor.
      motor_kd: The D gain of the motor.
      remove_default_joint_damping: Whether to remove the default joint damping.
      render: Whether to render the simulation.
      num_steps_to_log: The max number of control steps in one episode. If the
        number of steps is over num_steps_to_log, the environment will still
        be running, but only first num_steps_to_log will be recorded in logging.
      env_randomizer: An instance (or a list) of EnvRanzomier(s) that can
        randomize the environment during when env.reset() is called and add
        perturbations when env.step() is called.
      log_path: The path to write out logs. For the details of logging, refer to
        minitaur_logging.proto.
    """
    # _swing_offset and _extension_offset is to mimick the bent legs.
    self._swing_offset = np.zeros(NUM_LEGS)
    self._extension_offset = np.zeros(NUM_LEGS)
    super(MinitaurAlternatingLegsEnv,
          self).__init__(urdf_version=urdf_version,
                         accurate_motor_model_enabled=True,
                         motor_overheat_protection=True,
                         hard_reset=False,
                         motor_kp=motor_kp,
                         motor_kd=motor_kd,
                         remove_default_joint_damping=remove_default_joint_damping,
                         control_latency=control_latency,
                         pd_latency=pd_latency,
                         on_rack=on_rack,
                         render=render,
                         num_steps_to_log=num_steps_to_log,
                         env_randomizer=env_randomizer,
                         log_path=log_path,
                         control_time_step=control_time_step,
                         action_repeat=action_repeat)

    action_dim = 8
    action_high = np.array([0.1] * action_dim)
    self.action_space = spaces.Box(-action_high, action_high)

    self._cam_dist = 1.0
    self._cam_yaw = 30
    self._cam_pitch = -30

  def reset(self):
    self.desired_pitch = DESIRED_PITCH
    # In this environment, the actions are
    # [swing leg 1, swing leg 2, swing leg 3, swing leg 4,
    #  extension leg 1, extension leg 2, extension leg 3, extension leg 4]
    init_pose = [
        INIT_SWING_POS + self._swing_offset[0], INIT_SWING_POS + self._swing_offset[1],
        INIT_SWING_POS + self._swing_offset[2], INIT_SWING_POS + self._swing_offset[3],
        INIT_EXTENSION_POS + self._extension_offset[0],
        INIT_EXTENSION_POS + self._extension_offset[1],
        INIT_EXTENSION_POS + self._extension_offset[2],
        INIT_EXTENSION_POS + self._extension_offset[3]
    ]
    initial_motor_angles = self._convert_from_leg_model(init_pose)
    super(MinitaurAlternatingLegsEnv, self).reset(initial_motor_angles=initial_motor_angles,
                                                  reset_duration=0.5)
    return self._get_observation()

  def _convert_from_leg_model(self, leg_pose):
    motor_pose = np.zeros(NUM_MOTORS)
    for i in range(NUM_LEGS):
      motor_pose[2 * i] = leg_pose[NUM_LEGS + i] - (-1)**(i / 2) * leg_pose[i]
      motor_pose[2 * i + 1] = leg_pose[NUM_LEGS + i] + (-1)**(i / 2) * leg_pose[i]
    return motor_pose

  def _signal(self, t):
    initial_pose = np.array([
        INIT_SWING_POS, INIT_SWING_POS, INIT_SWING_POS, INIT_SWING_POS, INIT_EXTENSION_POS,
        INIT_EXTENSION_POS, INIT_EXTENSION_POS, INIT_EXTENSION_POS
    ])
    amplitude = STEP_AMPLITUDE
    period = STEP_PERIOD
    extension = amplitude * (-1.0 + math.cos(2 * math.pi / period * t))
    ith_leg = int(t / period) % 2
    first_leg = np.array([0, 0, 0, 0, 0, extension, extension, 0])
    second_leg = np.array([0, 0, 0, 0, extension, 0, 0, extension])
    if ith_leg:
      signal = initial_pose + second_leg
    else:
      signal = initial_pose + first_leg
    return signal

  def _transform_action_to_motor_command(self, action):
    #  Add swing_offset and extension_offset to mimick the bent legs.
    action[0:4] += self._swing_offset
    action[4:8] += self._extension_offset
    action += self._signal(self.minitaur.GetTimeSinceReset())
    action = self._convert_from_leg_model(action)
    return action

  def is_fallen(self):
    """Decide whether the minitaur has fallen.

    If the up directions between the base and the world is large (the dot
    product is smaller than 0.85), the minitaur is considered fallen.

    Returns:
      Boolean value that indicates whether the minitaur has fallen.
    """
    orientation = self.minitaur.GetBaseOrientation()
    rot_mat = self._pybullet_client.getMatrixFromQuaternion(orientation)
    local_up = rot_mat[6:]
    return np.dot(np.asarray([0, 0, 1]), np.asarray(local_up)) < 0.85

  def _reward(self):
    return 1.0

  def _get_true_observation(self):
    """Get the true observations of this environment.

    It includes the roll, the error between current pitch and desired pitch,
    roll dot and pitch dot of the base.

    Returns:
      The observation list.
    """
    observation = []
    roll, pitch, _ = self.minitaur.GetTrueBaseRollPitchYaw()
    roll_rate, pitch_rate, _ = self.minitaur.GetTrueBaseRollPitchYawRate()
    observation.extend([roll, pitch, roll_rate, pitch_rate])
    observation[1] -= self.desired_pitch  # observation[1] is the pitch
    self._true_observation = np.array(observation)
    return self._true_observation

  def _get_observation(self):
    observation = []
    roll, pitch, _ = self.minitaur.GetBaseRollPitchYaw()
    roll_rate, pitch_rate, _ = self.minitaur.GetBaseRollPitchYawRate()
    observation.extend([roll, pitch, roll_rate, pitch_rate])
    observation[1] -= self.desired_pitch  # observation[1] is the pitch
    self._observation = np.array(observation)
    return self._observation

  def _get_observation_upper_bound(self):
    """Get the upper bound of the observation.

    Returns:
      The upper bound of an observation. See GetObservation() for the details
        of each element of an observation.
    """
    upper_bound = np.zeros(self._get_observation_dimension())
    upper_bound[0:2] = 2 * math.pi  # Roll, pitch, yaw of the base.
    upper_bound[2:4] = 2 * math.pi / self._time_step  # Roll, pitch, yaw rate.
    return upper_bound

  def _get_observation_lower_bound(self):
    lower_bound = -self._get_observation_upper_bound()
    return lower_bound

  def set_swing_offset(self, value):
    """Set the swing offset of each leg.

    It is to mimic the bent leg.

    Args:
      value: A list of four values.
    """
    self._swing_offset = value

  def set_extension_offset(self, value):
    """Set the extension offset of each leg.

    It is to mimic the bent leg.

    Args:
      value: A list of four values.
    """
    self._extension_offset = value

  def set_desired_pitch(self, value):
    """Set the desired pitch of the base, which is a user input.

    Args:
      value: A scalar.
    """
    self.desired_pitch = value
