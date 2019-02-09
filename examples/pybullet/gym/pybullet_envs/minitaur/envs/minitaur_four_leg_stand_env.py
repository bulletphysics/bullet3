"""This file implements the gym environment of minitaur standing with four legs.

"""
import math
import operator
import random
import time
from gym import spaces
import numpy as np
from pybullet_envs.minitaur.envs import minitaur_gym_env
from pybullet_envs.minitaur.envs import minitaur_logging

INIT_EXTENSION_POS = 2.0
INIT_SWING_POS = 0.0
DESIRED_PITCH = 0
NUM_LEGS = 4
NUM_MOTORS = 2 * NUM_LEGS
STEP_PERIOD = 1.0 / 3.0  # Three steps per second.
STEP_AMPLITUDE = 0.75
PERTURBATION_TOTAL_STEP = 100
MOVING_FLOOR_TOTAL_STEP = 100
DERPY_V0_URDF_VERSION = minitaur_gym_env.DERPY_V0_URDF_VERSION
RAINBOW_DASH_V0_URDF_VERSION = minitaur_gym_env.RAINBOW_DASH_V0_URDF_VERSION


class MinitaurFourLegStandEnv(minitaur_gym_env.MinitaurGymEnv):
  """The gym environment for the minitaur.

  It simulates the a minitaur standing with four legs. The state space
  include the orientation of the torso, and the action space is the desired
  motor angle for each motor. The reward function is based on how close the
  action to zero and the height of the robot base. It prefers a similar pose to
  the signal while keeping balance.
  """
  metadata = {
      "render.modes": ["human", "rgb_array"],
      "video.frames_per_second": 166
  }

  def __init__(self,
               urdf_version=None,
               hard_reset=True,
               remove_default_joint_damping=True,
               control_latency=0.0,
               pd_latency=0.0,
               on_rack=False,
               motor_kp=1.0,
               motor_kd=0.02,
               render=False,
               env_randomizer=None,
               use_angular_velocity_in_observation=False,
               use_motor_angle_in_observation=False,
               control_time_step=0.006,
               action_repeat=6,
               log_path=None):
    """Initialize the minitaur alternating legs gym environment.

    Args:
      urdf_version: [DEFAULT_URDF_VERSION, DERPY_V0_URDF_VERSION,
        RAINBOW_DASH_V0_URDF_VERSION] are allowable
        versions. If None, DEFAULT_URDF_VERSION is used. DERPY_V0_URDF_VERSION
        is the result of first pass system identification for derpy.
        We will have a different URDF and related Minitaur class each time we
        perform system identification. While the majority of the code of the
        class remains the same, some code changes (e.g. the constraint location
        might change). __init__() will choose the right Minitaur class from
        different minitaur modules based on urdf_version.
      hard_reset: Whether to wipe the simulation and load everything when reset
        is called. If set to false, reset just place the minitaur back to start
        position and set its pose to initial configuration.
      remove_default_joint_damping: Whether to remove the default joint damping.
      control_latency: It is the delay in the controller between when an
        observation is made at some point, and when that reading is reported
        back to the Neural Network.
      pd_latency: latency of the PD controller loop. PD calculates PWM based on
        the motor angle and velocity. The latency measures the time between when
        the motor angle and velocity are observed on the microcontroller and
        when the true state happens on the motor. It is typically (0.001-
        0.002s).
      on_rack: Whether to place the minitaur on rack. This is only used to debug
        the walking gait. In this mode, the minitaur's base is hung midair so
        that its walking gait is clearer to visualize.
      motor_kp: The P gain of the motor.
      motor_kd: The D gain of the motor.
      render: Whether to render the simulation.
      env_randomizer: An instance (or a list) of EnvRanzomier(s) that can
        randomize the environment during when env.reset() is called and add
        perturbations when env.step() is called.
      use_angular_velocity_in_observation: Whether to include roll_dot and
        pitch_dot of the base in the observation.
      use_motor_angle_in_observation: Whether to include motor angles in the
        observation.
      control_time_step: The time step between two successive control signals.
      action_repeat: The number of simulation steps before actions are applied.
      log_path: The path to write out logs. For the details of logging, refer to
        minitaur_logging.proto.
    """
    # _swing_offset and _extension_offset is to mimic the motor zero-calibration
    # errors.
    self._swing_offset = np.zeros(NUM_LEGS)
    self._extension_offset = np.zeros(NUM_LEGS)
    self._use_angular_velocity_in_observation = use_motor_angle_in_observation
    self._use_motor_angle_in_observation = use_motor_angle_in_observation
    super(MinitaurFourLegStandEnv, self).__init__(
        urdf_version=urdf_version,
        control_time_step=control_time_step,
        action_repeat=action_repeat,
        remove_default_joint_damping=remove_default_joint_damping,
        accurate_motor_model_enabled=True,
        motor_overheat_protection=True,
        hard_reset=hard_reset,
        motor_kp=motor_kp,
        motor_kd=motor_kd,
        control_latency=control_latency,
        pd_latency=pd_latency,
        on_rack=on_rack,
        render=render,
        env_randomizer=env_randomizer,
        reflection = False,
        log_path=log_path)

    action_dim = 4
    action_low = np.array([-1.0] * action_dim)
    action_high = -action_low
    self.action_space = spaces.Box(action_low, action_high)

    self._cam_dist = 1.0
    self._cam_yaw = 30
    self._cam_pitch = -30
    self._perturbation_magnitude = 0.0
    self._sign = 1.0
    self._cur_ori = [0, 0, 0, 1]
    self._goal_ori = [0, 0, 0, 1]

  def reset(self):
    self.desired_pitch = DESIRED_PITCH
    # In this environment, the actions are
    # [swing leg 1, swing leg 2, swing leg 3, swing leg 4,
    #  extension leg 1, extension leg 2, extension leg 3, extension leg 4]
    init_pose = [
        INIT_SWING_POS + self._swing_offset[0],
        INIT_SWING_POS + self._swing_offset[1],
        INIT_SWING_POS + self._swing_offset[2],
        INIT_SWING_POS + self._swing_offset[3],
        INIT_EXTENSION_POS + self._extension_offset[0],
        INIT_EXTENSION_POS + self._extension_offset[1],
        INIT_EXTENSION_POS + self._extension_offset[2],
        INIT_EXTENSION_POS + self._extension_offset[3]
    ]
    initial_motor_angles = self._convert_from_leg_model(init_pose)
    self._pybullet_client.resetBasePositionAndOrientation(
        0, [0, 0, 0], [0, 0, 0, 1])
    super(MinitaurFourLegStandEnv, self).reset(
        initial_motor_angles=initial_motor_angles, reset_duration=0.5)
    return self._get_observation()

  def step(self, action):
    """Step forward the simulation, given the action.

    Args:
      action: A list of desired motor angles for eight motors.

    Returns:
      observations: Roll, pitch of the base, and roll, pitch rate.
      reward: The reward for the current state-action pair.
      done: Whether the episode has ended.
      info: A dictionary that stores diagnostic information.

    Raises:
      ValueError: The action dimension is not the same as the number of motors.
      ValueError: The magnitude of actions is out of bounds.
    """
    if self._is_render:
      # Sleep, otherwise the computation takes less time than real time,
      # which will make the visualization like a fast-forward video.
      time_spent = time.time() - self._last_frame_time
      self._last_frame_time = time.time()
      time_to_sleep = self.control_time_step - time_spent
      if time_to_sleep > 0:
        time.sleep(time_to_sleep)
      base_pos = self.minitaur.GetBasePosition()
      # Keep the previous orientation of the camera set by the user.
      [yaw, pitch,
       dist] = self._pybullet_client.getDebugVisualizerCamera()[8:11]
      self._pybullet_client.resetDebugVisualizerCamera(dist, yaw, pitch,
                                                       base_pos)
    action = self._transform_action_to_motor_command(action)
    t = self._env_step_counter % MOVING_FLOOR_TOTAL_STEP
    if t == 0:
      self.seed()
      orientation_x = random.uniform(-0.2, 0.2)
      self.seed()
      orientation_y = random.uniform(-0.2, 0.2)
      _, self._cur_ori = self._pybullet_client.getBasePositionAndOrientation(0)
      self._goal_ori = self._pybullet_client.getQuaternionFromEuler(
          [orientation_x, orientation_y, 0])
    t = float(float(t) / float(MOVING_FLOOR_TOTAL_STEP))
    ori = map(operator.add, [x * (1.0 - t) for x in self._cur_ori],
              [x * t for x in self._goal_ori])
    ori=list(ori)
    print("ori=",ori)
    self._pybullet_client.resetBasePositionAndOrientation(0, [0, 0, 0], ori)
    if self._env_step_counter % PERTURBATION_TOTAL_STEP == 0:
      self._perturbation_magnitude = random.uniform(0.0, 0.0)
      if self._sign < 0.5:
        self._sign = 1.0
      else:
        self._sign = 0.0
    self._pybullet_client.applyExternalForce(
        objectUniqueId=1,
        linkIndex=-1,
        forceObj=[self._sign * self._perturbation_magnitude, 0.0, 0.0],
        posObj=[0.0, 0.0, 0.0],
        flags=self._pybullet_client.LINK_FRAME)
    self.minitaur.Step(action)
    self._env_step_counter += 1
    done = self._termination()
    obs = self._get_true_observation()
    reward = self._reward()
    if self._log_path is not None:
      minitaur_logging.update_episode_proto(self._episode_proto, self.minitaur,
                                            action, self._env_step_counter)
    if done:
      self.minitaur.Terminate()
    return np.array(self._get_observation()), reward, done, {}

  def _convert_from_leg_model(self, leg_pose):
    motor_pose = np.zeros(NUM_MOTORS)
    for i in range(NUM_LEGS):
      motor_pose[2 * i] = leg_pose[NUM_LEGS + i] - (-1)**(i / 2) * leg_pose[i]
      motor_pose[2 * i
                 + 1] = leg_pose[NUM_LEGS + i] + (-1)**(i / 2) * leg_pose[i]
    return motor_pose

  def _signal(self, t):
    initial_pose = np.array([
        INIT_SWING_POS, INIT_SWING_POS, INIT_SWING_POS, INIT_SWING_POS,
        INIT_EXTENSION_POS, INIT_EXTENSION_POS, INIT_EXTENSION_POS,
        INIT_EXTENSION_POS
    ])
    signal = initial_pose
    return signal

  def _transform_action_to_motor_command(self, action):
    # Add swing_offset and extension_offset to mimick the motor zero-calibration
    # errors.
    real_action = np.array([0.0] * 8)
    real_action[4:8] = action + self._extension_offset
    real_action[0:4] = self._swing_offset
    real_action += self._signal(self.minitaur.GetTimeSinceReset())
    real_action = self._convert_from_leg_model(real_action)
    return real_action

  def is_fallen(self):
    """Decide whether the minitaur has fallen.

    # TODO(yunfeibai): choose the fallen option for force perturbation and
    moving floor, and update the comments.

    If the up directions between the base and the world is large (the dot
    product is smaller than 0.85), or the robot base is lower than 0.24, the
    minitaur is considered fallen.

    Returns:
      Boolean value that indicates whether the minitaur has fallen.
    """
    orientation = self.minitaur.GetBaseOrientation()
    rot_mat = self._pybullet_client.getMatrixFromQuaternion(orientation)
    local_up = rot_mat[6:]
    _, _, height = self.minitaur.GetBasePosition()
    local_global_up_dot_product = np.dot(
        np.asarray([0, 0, 1]), np.asarray(local_up))
    return local_global_up_dot_product < 0.85 or height < 0.15

  def _reward(self):
    roll, pitch, _ = self.minitaur.GetBaseRollPitchYaw()
    return 1.0 / (0.001 + math.fabs(roll) + math.fabs(pitch))

  def _get_observation(self):
    """Get the true observations of this environment.

    It includes the roll, pitch, roll dot, pitch dot of the base, and the motor
    angles.

    Returns:
      The observation list.
    """
    roll, pitch, _ = self.minitaur.GetBaseRollPitchYaw()
    observation = [roll, pitch]
    if self._use_angular_velocity_in_observation:
      roll_rate, pitch_rate, _ = self.minitaur.GetBaseRollPitchYawRate()
      observation.extend([roll_rate, pitch_rate])
    if self._use_motor_angle_in_observation:
      observation.extend(self.minitaur.GetMotorAngles().tolist())
    self._observation = np.array(observation)
    return self._observation

  def _get_true_observation(self):
    """Get the true observations of this environment.

    It includes the roll, pitch, roll dot, pitch dot of the base, and the motor
    angles.

    Returns:
      The observation list.
    """
    roll, pitch, _ = self.minitaur.GetBaseRollPitchYaw()
    observation = [roll, pitch]
    if self._use_angular_velocity_in_observation:
      roll_rate, pitch_rate, _ = self.minitaur.GetBaseRollPitchYawRate()
      observation.extend([roll_rate, pitch_rate])
    if self._use_motor_angle_in_observation:
      observation.extend(self.minitaur.GetMotorAngles().tolist())

    self._observation = np.array(observation)
    return self._observation

  def _get_observation_upper_bound(self):
    """Get the upper bound of the observation.

    Returns:
      The upper bound of an observation. See GetObservation() for the details
        of each element of an observation.
    """
    upper_bound = [2 * math.pi] * 2  # Roll, pitch the base.
    if self._use_angular_velocity_in_observation:
      upper_bound.extend([2 * math.pi / self._time_step] * 2)
    if self._use_motor_angle_in_observation:
      upper_bound.extend([2 * math.pi] * 8)
    return np.array(upper_bound)

  def _get_observation_lower_bound(self):
    lower_bound = -self._get_observation_upper_bound()
    return lower_bound

  def set_swing_offset(self, value):
    """Set the swing offset of each leg.

    It is to mimic the motor zero-calibration errors.

    Args:
      value: A list of four values.
    """
    self._swing_offset = value

  def set_extension_offset(self, value):
    """Set the extension offset of each leg.

    It is to mimic the motor zero-calibration errors.

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
