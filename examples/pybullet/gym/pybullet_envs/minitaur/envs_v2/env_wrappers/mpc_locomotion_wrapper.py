"""An env that uses MPC-based motion controller to realize higher level footstep planning."""

import copy
import enum
from typing import Sequence

import dataclasses
import gin
import gym
import numpy as np

from pybullet_envs.minitaur.agents.baseline_controller import com_height_estimator
from pybullet_envs.minitaur.agents.baseline_controller import gait_generator as gait_generator_lib
from pybullet_envs.minitaur.agents.baseline_controller import imu_based_com_velocity_estimator
from pybullet_envs.minitaur.agents.baseline_controller import multi_state_estimator
from pybullet_envs.minitaur.agents.baseline_controller import openloop_gait_generator
from pybullet_envs.minitaur.agents.baseline_controller import torque_stance_leg_controller
from pybullet_envs.minitaur.robots import laikago_kinematic_constants

_UNIT_QUATERNION = (0, 0, 0, 1)
_NUM_LEGS = laikago_kinematic_constants.NUM_LEGS
_MOTORS_PER_LEG = 3
_DEFAULT_BODY_HEIGHT = 0.45
_DEFAULT_BASE_SPEED = (0.0, 0.0)
_DEFAULT_BASE_TWIST_SPEED = 0.0
_DEFAULT_ROLL_PITCH = (0.0, 0.0)
_DEFAULT_SWING_TARGET = (0.0, 0.0, 0.0)
_DEFAULT_SWING_CLERANCE = 0.05
_MOTOR_KP = [220.0] * 12
_MOTOR_KD = [0.3, 2.0, 2.0] * 4
_BASE_VELOCITY_ACTION_RANGE = ((-1.0, -0.1), (1.0, 0.1))
_BASE_TWIST_SPEED_ACTION_RANGE = (-0.5, 0.5)
_BASE_HEIGHT_ACTION_RANGE = (0.3, 0.5)
# Action bound for the swing target in local x, y, z direction.
_SWING_TARGET_ACTION_RANGE = ((-0.3, -0.1, -0.25), (0.3, 0.1, 0.25))
_PITCH_ROLL_ACTION_RANGE = (-0.35, 0.35)
_SWING_CLEARANCE_ACTION_RANGE = (0.05, 0.3)
_SWING_TARGET_DELTA_ACTION_RANGE = ((-0.02, -0.01, -0.02), (0.02, 0.01, 0.02))
_SWING_CLEARANCE_DELTA_RANGE = (-0.02, 0.02)


@gin.configurable
@dataclasses.dataclass
class BaseTargetHorizontalComVelocityHeuristic(object):
  """A class for mapping swing foot targets to a heuristic target com velocity."""
  horizontal_com_velocity_heuristic: np.ndarray = np.zeros(2)

  def update_horizontal_com_velocity_heuristic(self, hip_relative_swing_targets,
                                               com_velocity, swing_durations):
    del hip_relative_swing_targets, com_velocity, swing_durations
    pass

  def reset(self):
    self.horizontal_com_velocity_heuristic = np.zeros(2)


# TODO(magicmelon): Add a one-pager to explain the inverse raibert heuristics.
@gin.configurable
class InverseRaibertTargetHorizontalComVelocityHeuristic(
    BaseTargetHorizontalComVelocityHeuristic):
  """A class for mapping swing foot targets to a target com velocity with Raibert Heuristics."""

  def __init__(self, gains=(-0.25, -0.1)):
    self._gains = np.array(gains)

  def update_horizontal_com_velocity_heuristic(self, hip_relative_swing_targets,
                                               com_velocity, swing_durations):
    assert len(hip_relative_swing_targets) == len(swing_durations)
    target_com_horizontal_velocities = []
    for i in range(len(hip_relative_swing_targets)):
      target_com_horizontal_velocity = (
          com_velocity / 2.0 * swing_durations[i] -
          hip_relative_swing_targets[i]) / self._gains + com_velocity
      target_com_horizontal_velocities.append(target_com_horizontal_velocity)
    if target_com_horizontal_velocities:
      self.horizontal_com_velocity_heuristic = np.mean(
          target_com_horizontal_velocities, axis=0)


@gin.constants_from_enum
class Gait(enum.Enum):
  """The possible gaits."""
  WALK = 0
  TROT = 1


def _select_gait(gait_type=Gait.WALK):
  """Selects a gait pattern.

  Args:
    gait_type: which gait to use.

  Returns:
    A tuple of (stance_duration, duty_factor, initial_phase)
  """
  # Each gait is composed of stance_duration, duty_factor, and
  # init_phase_full_cycle.
  if gait_type == Gait.TROT:
    return [0.3] * 4, [0.6] * 4, [0, 0.5, 0.5, 0]
  elif gait_type == Gait.WALK:
    return [0.75] * 4, [0.8] * 4, [0.25, 0.75, 0.5, 0]
  else:
    raise NotImplementedError


@gin.configurable
class MPCLocomotionWrapper(object):
  """An env that uses MPC-based motion controller to realize footstep planning.

    The env takes as input the target position of the swing feet and the target
    base movements, and internally uses an MPC-based controller to achieve these
    targets. It assumes that the robot follows a given gait pattern, specified
    during initialization.
    Before each foot starts to swing, the env will request from the policy a
    target swing location and height defined in the local frame w.r.t the
    default toe position. During the swing of the foot, the policy can adjust
    the base velocity, height, roll, pitch, and twist. Optionally, the policy
    can also output a delta to the last target swing location to adjust the
    swing trajectory during the swing
    Observations (introduced in this env):
      gait_phases (4D): Normalized phase within the gait cycle for each foot.
      feet_states (4D): State of each foot. -1: stance, 1: swing, -2: lose
        contact, 2: early contact.
      need_new_swing_target (4D): Whether the foot needs a new swing target at
        the current step. Set to 1 when the foot switches to swing from a
        different state. When equals to 0, the corresponding foot target will
        not have effect.
      estimated_base_speed (3D): Estimated base velocity.
      estimated_body_height (1D): Estimated base height.
      heuristics_com_velocity (3D): Target base velocity calculated from the
        input step-length using inverse Raibert heuristics. Used when
        compute_heuristic_com_speed is True.
      current_toe_target: Immediate tracking targets for the four feet in the
        local frame.
    Action components:
      swing_targets (12D): Used in HL_LL and HL_only mode. Specifies
        the swing target for each foot w.r.t the default local toe position.
      swing_clearance (4D): Used in HL_LL and HL_only mode. Specifies
        the height of the highest point in the swing trajectory.
      swing_targets_delta (12D): Used if policy_output_swing_action_delta.
        Specifies the change in the swing target for each foot.
      swing_clearance_delta (4D): Used if policy_output_swing_action_delta.
        Specifies the change in the swing clearance for each foot.
      target_base_velocity (2D): Target base velocity in the horizontal plane.
        Used when compute_heuristic_com_speed is False.
      base_twist (1D): Target base twist.
      base_height (1D): Target base height.
      base_roll_pitch (2D): Target base roll and pitch.

  Attributes:
    observation: The current observation of the environment.
    last_action: The last that was used to step the environment.
    env_step_counter: The number of control steps that have been elapesed since
      the environment is reset.
    action_space: The action space of the environment.
  """

  def __init__(
      self,
      gym_env,
      swing_target_action_range=_SWING_TARGET_ACTION_RANGE,
      swing_clearance_action_range=_SWING_CLEARANCE_ACTION_RANGE,
      pitch_action_range=_PITCH_ROLL_ACTION_RANGE,
      roll_action_range=_PITCH_ROLL_ACTION_RANGE,
      base_velocity_action_range=_BASE_VELOCITY_ACTION_RANGE,
      base_twist_action_range=_BASE_TWIST_SPEED_ACTION_RANGE,
      base_height_action_range=_BASE_HEIGHT_ACTION_RANGE,
      policy_output_swing_action_delta=False,
      swing_target_delta_range=_SWING_TARGET_DELTA_ACTION_RANGE,
      swing_clearance_delta_range=_SWING_CLEARANCE_DELTA_RANGE,
      foot_friction_coeff=0.5,
      contact_detection_force_threshold=0.0,
      locomotion_gait=Gait.WALK,
      target_horizontal_com_velocity_heuristic=BaseTargetHorizontalComVelocityHeuristic(
      ),
      robot_mass_in_mpc=235.0 / 9.8,
      control_frequency=20,
      com_velocity_estimator_class=imu_based_com_velocity_estimator
      .IMUBasedCOMVelocityEstimator):
    """Initializes the wrapper.

    Args:
      gym_env: the wrapped gym environment.
      swing_target_action_range: range for the swing targets specified before
        each swing.
      swing_clearance_action_range: range for the swing clearance.
      pitch_action_range: range for the target body pitch.
      roll_action_range: range for the target body roll.
      base_velocity_action_range: range for the base velocity.
      base_twist_action_range: range for the base twist.
      base_height_action_range: range for the base height.
      policy_output_swing_action_delta: whether to allow the policy to output an
        adjustment to the last swing target during the swing motion.
      swing_target_delta_range: range for the adjustment of the swing target.
      swing_clearance_delta_range: range for the swing clearance adjustments.
      foot_friction_coeff: friction on the feet.
      contact_detection_force_threshold: Threshold of the contact sensor for
        determining whether a foot is in contact. Use 20 for real robot and 0
        for simulation.
      locomotion_gait: Gait to be used.
      target_horizontal_com_velocity_heuristic: .
      robot_mass_in_mpc: mass of the robot used in MPC.
      control_frequency: frequency of querying the policy. The internal MPC
        controller can have higher frequency. Note that the policy outputs a
        swing target and clearance at each query, however, it is only used by
        the environment at the beginning of each swing phase (when
        need_new_swing_target is 1).
      com_velocity_estimator_class: class of the com velocity estimator. Use
        IMUBasedCOMVelocityEstimator for estimating velocity from IMU sensor and
        contact states. Use COMVelocityEstimator for using the ground-truth com
        velocity (e.g. when mocap is available).
    """
    self._gym_env = gym_env
    self._time_per_control_step = 1.0 / control_frequency
    self._foot_friction_coeff = foot_friction_coeff
    self._contact_detection_force_threshold = contact_detection_force_threshold
    self._locomotion_gait = locomotion_gait
    self._policy_output_swing_action_delta = policy_output_swing_action_delta
    self._target_horizontal_com_velocity_heuristic = target_horizontal_com_velocity_heuristic

    self.last_action = None

    self._configure_action_space(
        swing_target_action_range, swing_clearance_action_range,
        pitch_action_range, roll_action_range, base_velocity_action_range,
        base_twist_action_range, base_height_action_range,
        policy_output_swing_action_delta, swing_target_delta_range,
        swing_clearance_delta_range)

    self._configure_observation_space()

    # Set up the MPC controller
    stance_duration, duty_factor, initial_phase = _select_gait(locomotion_gait)
    self._gait_generator = openloop_gait_generator.OpenloopGaitGenerator(
        self._gym_env.robot, stance_duration, duty_factor, initial_phase,
        contact_detection_force_threshold)
    self._com_velocity_estimator = com_velocity_estimator_class(
        self._gym_env.robot)
    self._com_height_estimator = com_height_estimator.COMHeightEstimator(
        self._gym_env.robot)
    self._state_estimator = multi_state_estimator.MultiStateEstimator(
        self._gym_env.robot,
        state_estimators=[
            self._com_velocity_estimator, self._com_height_estimator
        ])
    self._stance_controller = torque_stance_leg_controller.TorqueStanceLegController(
        self._gym_env.robot,
        self._gait_generator,
        self._state_estimator,
        desired_speed=np.array(_DEFAULT_BASE_SPEED),
        desired_twisting_speed=_DEFAULT_BASE_TWIST_SPEED,
        desired_body_height=_DEFAULT_BODY_HEIGHT,
        desired_roll_pitch=np.array(_DEFAULT_ROLL_PITCH),
        body_mass=robot_mass_in_mpc)

  def _configure_observation_space(self):
    """Configure the observation space."""
    self.observation_space.spaces["gait_phases"] = gym.spaces.Box(
        np.array([-1.0] * 4), np.array([1.0] * 4))
    self.observation_space.spaces["feet_states"] = gym.spaces.Box(
        np.array([-2.0] * 4), np.array([2.0] * 4))
    self.observation_space.spaces["need_new_swing_target"] = gym.spaces.Box(
        np.array([0.0] * 4), np.array([1.0] * 4))
    self.observation_space.spaces["estimated_base_speed"] = gym.spaces.Box(
        np.array([-1.0, -1.0, -1.0]), np.array([1.0, 1.0, 1.0]))
    self.observation_space.spaces["estimated_body_height"] = gym.spaces.Box(
        np.array([0.35]), np.array([0.5]))
    self.observation_space.spaces["heuristics_com_velocity"] = gym.spaces.Box(
        np.array([-1.0] * 2), np.array([1.0] * 2))
    self.observation_space.spaces["current_toe_target"] = gym.spaces.Box(
        np.array([-1.0, -1.0, -1.0] * _NUM_LEGS),
        np.array([1.0, 1.0, 1.0] * _NUM_LEGS))

    # Needed so that LastActionSensor uses the correct action space.
    for s in self.all_sensors():
      s.on_reset(self)
    for sensor in self.all_sensors():
      if sensor.get_name() not in self._gym_config.ignored_sensor_list:
        if hasattr(sensor, "observation_space"):
          self.observation_space.spaces[
              sensor.get_name()] = sensor.observation_space

    self.task.reset(self)
    if hasattr(self.task, "observation_space"):
      self.observation_space.spaces[
          self.task.get_name()] = self.task.observation_space

  def _configure_action_space(self, swing_target_action_range,
                              swing_clearance_action_range, pitch_action_range,
                              roll_action_range, base_velocity_action_range,
                              base_twist_action_range, base_height_action_range,
                              policy_output_swing_action_delta,
                              swing_target_delta_range,
                              swing_clearance_delta_range):
    """Configure the action space."""
    ac_lb = np.array([])
    ac_ub = np.array([])

    # Index of different part of the actions within the action array.
    self._action_start_indices = {}
    self._action_dimensions = {}
    self._action_names = []

    # Swing targets and swing clearance.
    for leg_id in range(_NUM_LEGS):
      self._action_start_indices["swing_targets_" + str(leg_id)] = len(ac_lb)
      self._action_dimensions["swing_targets_" + str(leg_id)] = len(
          swing_target_action_range[0])
      self._action_names.append("swing_targets_" + str(leg_id))
      ac_lb = np.concatenate([ac_lb, swing_target_action_range[0]])
      ac_ub = np.concatenate([ac_ub, swing_target_action_range[1]])
    for leg_id in range(_NUM_LEGS):
      self._action_start_indices["swing_clearance_" + str(leg_id)] = len(ac_lb)
      self._action_dimensions["swing_clearance_" + str(leg_id)] = 1
      self._action_names.append("swing_clearance_" + str(leg_id))
      ac_lb = np.concatenate([ac_lb, [swing_clearance_action_range[0]]])
      ac_ub = np.concatenate([ac_ub, [swing_clearance_action_range[1]]])

    # Delta to the swing targets and clearance.
    if policy_output_swing_action_delta:
      for leg_id in range(_NUM_LEGS):
        self._action_start_indices["swing_targets_delta_" +
                                   str(leg_id)] = len(ac_lb)
        self._action_dimensions["swing_targets_delta_" + str(leg_id)] = len(
            swing_target_delta_range[0])
        self._action_names.append("swing_targets_delta_" + str(leg_id))
        ac_lb = np.concatenate([ac_lb, swing_target_delta_range[0]])
        ac_ub = np.concatenate([ac_ub, swing_target_delta_range[1]])
      for leg_id in range(_NUM_LEGS):
        self._action_start_indices["swing_clearance_delta_" +
                                   str(leg_id)] = len(ac_lb)
        self._action_dimensions["swing_clearance_delta_" + str(leg_id)] = 1
        self._action_names.append("swing_clearance_delta_" + str(leg_id))
        ac_lb = np.concatenate([ac_lb, [swing_clearance_delta_range[0]]])
        ac_ub = np.concatenate([ac_ub, [swing_clearance_delta_range[1]]])

    # Desired CoM velocity actions
    # Do not include the action if bounds are all equal to zero
    if not np.all(np.array(base_velocity_action_range) == 0):
      self._action_start_indices["target_base_velocity"] = len(ac_lb)
      self._action_dimensions["target_base_velocity"] = 2
      self._action_names.append("target_base_velocity")
      ac_lb = np.concatenate([
          ac_lb,
          [base_velocity_action_range[0][0], base_velocity_action_range[0][1]]
      ])
      ac_ub = np.concatenate([
          ac_ub,
          [base_velocity_action_range[1][0], base_velocity_action_range[1][1]]
      ])

    # Base twist speed action
    self._action_start_indices["base_twist"] = len(ac_lb)
    self._action_dimensions["base_twist"] = 1
    self._action_names.append("base_twist")
    ac_lb = np.concatenate([ac_lb, [base_twist_action_range[0]]])
    ac_ub = np.concatenate([ac_ub, [base_twist_action_range[1]]])

    # Base height action
    self._action_start_indices["base_height"] = len(ac_lb)
    self._action_dimensions["base_height"] = 1
    self._action_names.append("base_height")
    ac_lb = np.concatenate([ac_lb, [base_height_action_range[0]]])
    ac_ub = np.concatenate([ac_ub, [base_height_action_range[1]]])

    # Roll-pitch action
    self._action_start_indices["base_roll_pitch"] = len(ac_lb)
    self._action_dimensions["base_roll_pitch"] = 2
    self._action_names.append("base_roll_pitch")
    ac_lb = np.concatenate(
        [ac_lb, [roll_action_range[0], pitch_action_range[0]]])
    ac_ub = np.concatenate(
        [ac_ub, [roll_action_range[1], pitch_action_range[1]]])

    self.action_space = gym.spaces.Box(ac_lb, ac_ub)

  def __getattr__(self, attr):
    return getattr(self._gym_env, attr)

  def _fill_observations(self, obs):
    """Fill the additional observations from this wrapper."""
    phase_offset = np.array([
        0 if leg_state == gait_generator_lib.LegState.STANCE else 1
        for leg_state in self._gait_generator.desired_leg_state
    ])
    obs["gait_phases"] = self._gait_generator.normalized_phase - phase_offset
    obs["feet_states"] = []
    for leg_state in self._gait_generator.desired_leg_state:
      if leg_state == gait_generator_lib.LegState.STANCE:
        obs["feet_states"].append(-1)
      if leg_state == gait_generator_lib.LegState.SWING:
        obs["feet_states"].append(1)
      if leg_state == gait_generator_lib.LegState.EARLY_CONTACT:
        obs["feet_states"].append(2)
      if leg_state == gait_generator_lib.LegState.LOSE_CONTACT:
        obs["feet_states"].append(-2)
    obs["need_new_swing_target"] = np.copy(self._need_new_swing_target)
    obs["estimated_base_speed"] = self._state_estimator.com_velocity_body_yaw_aligned_frame
    obs["estimated_body_height"] = [self._state_estimator.estimated_com_height]
    obs["heuristics_com_velocity"] = np.copy(
        self._target_horizontal_com_velocity_heuristic
        .horizontal_com_velocity_heuristic)
    obs["current_toe_target"] = np.copy(self._current_toe_target)

  def _reset_mpc_controller(self):
    """Reset the state of the MPC controller."""
    self._gait_generator.reset(0.0)
    self._state_estimator.reset(0.0)
    self._stance_controller.reset(0.0)
    self._stance_controller.desired_speed = np.array(_DEFAULT_BASE_SPEED)
    self._stance_controller.desired_twisting_speed = _DEFAULT_BASE_TWIST_SPEED
    self._stance_controller.desired_body_height = _DEFAULT_BODY_HEIGHT
    self._stance_controller.desired_roll_pitch = np.array(_DEFAULT_ROLL_PITCH)
    self._mpc_reset_time = self.robot.GetTimeSinceReset()

  def reset(self, **kwargs):
    """Reset the environment."""
    self._gym_env.reset(**kwargs)

    self._reset_mpc_controller()

    self._last_leg_state = copy.copy(self._gait_generator.leg_state)

    self._initial_local_toe_positions = np.array(
        self._gym_env.robot.foot_positions(position_in_world_frame=False))

    self._current_toe_target = np.reshape(
        copy.deepcopy(self._initial_local_toe_positions), 3 * _NUM_LEGS)

    # Record last lift off position for each foot for computing swing
    # trajectory.
    self._lift_off_positions = copy.deepcopy(self._initial_local_toe_positions)

    # Swing command by policy at the beginning of each swing phase.
    self._nominal_swing_leg_commands = []
    for _ in range(_NUM_LEGS):
      self._nominal_swing_leg_commands.append({
          "swing_target": np.array(_DEFAULT_SWING_TARGET),
          "swing_clearance": _DEFAULT_SWING_CLERANCE
      })
    # Actual swing leg commands that incorporated potential delta adjustments
    # from the policy.
    self._actual_swing_leg_commands = []
    for _ in range(_NUM_LEGS):
      self._actual_swing_leg_commands.append({
          "swing_target": np.array(_DEFAULT_SWING_TARGET),
          "swing_clearance": _DEFAULT_SWING_CLERANCE
      })

    self._need_new_swing_target = np.array([0.0] * _NUM_LEGS)

    self._target_horizontal_com_velocity_heuristic.reset()

    self.last_action = []
    for leg_id in range(_NUM_LEGS):
      self.last_action.extend(
          [0.0] * self._action_dimensions["swing_targets_" + str(leg_id)])
    for leg_id in range(_NUM_LEGS):
      self.last_action.extend(
          [0.0] * self._action_dimensions["swing_clearance_" + str(leg_id)])
    if self._policy_output_swing_action_delta:
      for leg_id in range(_NUM_LEGS):
        self.last_action.extend(
            [0.0] *
            self._action_dimensions["swing_targets_delta_" + str(leg_id)])
      for leg_id in range(_NUM_LEGS):
        self.last_action.extend(
            [0.0] *
            self._action_dimensions["swing_clearance_delta_" + str(leg_id)])
    print("=========", len(self.last_action))
    if "target_base_velocity" in self._action_names:
      self.last_action.extend([0.0] *
                              self._action_dimensions["target_base_velocity"])
    self.last_action.extend([0.0] * self._action_dimensions["base_twist"])
    self.last_action.extend([0.0] * self._action_dimensions["base_height"])
    self.last_action.extend([0.0] * self._action_dimensions["base_roll_pitch"])

    # Needed for LastActionSensor to use the correct last_action.
    for s in self.all_sensors():
      s.on_reset(self)
    obs = self._get_observation()
    self._fill_observations(obs)

    self.task.reset(self)
    self._step_counter = 0

    # Change feet friction.
    for link_id in list(
        self.robot.urdf_loader.get_end_effector_id_dict().values()):
      self.pybullet_client.changeDynamics(
          self.robot.robot_id,
          link_id,
          lateralFriction=self._foot_friction_coeff)
    self._observation = obs

    return self._observation

  def _clean_up_action(self, action):
    """Return a cleaned up action to use previous value or zero for components not used.

    Args:
      action: Input action from the policy.

    Returns:
      A cleaned up action where components not used in this step is replaced
        with zero or value from previous steps.
    """
    cleaned_action = np.copy(action)
    for leg_id in range(_NUM_LEGS):
      if not self._need_new_swing_target[leg_id]:
        swing_target_name = "swing_targets_" + str(leg_id)
        swing_targets_start_index = self._action_start_indices[
            swing_target_name]
        swing_targets_end_index = swing_targets_start_index + self._action_dimensions[
            swing_target_name]
        cleaned_action[swing_targets_start_index:
                       swing_targets_end_index] = self.last_action[
                           swing_targets_start_index:swing_targets_end_index]

        swing_clearance_name = "swing_clearance_" + str(leg_id)
        swing_clearance_start_index = self._action_start_indices[
            swing_clearance_name]
        cleaned_action[swing_clearance_start_index] = self.last_action[
            swing_clearance_start_index]
    if self._policy_output_swing_action_delta:
      for leg_id in range(_NUM_LEGS):
        if self._observation["feet_states"][leg_id] != 1:
          swing_target_delta_name = "swing_targets_delta_" + str(leg_id)
          swing_targets_delta_start_index = self._action_start_indices[
              swing_target_delta_name]
          swing_targets_delta_end_index = swing_targets_delta_start_index + self._action_dimensions[
              swing_target_delta_name]
          cleaned_action[
              swing_targets_delta_start_index:
              swing_targets_delta_end_index] = self.last_action[
                  swing_targets_delta_start_index:swing_targets_delta_end_index]

          swing_clearance_delta_name = "swing_clearance_delta_" + str(leg_id)
          swing_clearance_delta_start_index = self._action_start_indices[
              swing_clearance_delta_name]
          cleaned_action[swing_clearance_delta_start_index] = self.last_action[
              swing_clearance_delta_start_index]

    return cleaned_action

  def _get_toe_tracking_target(self, lift_off_position, phase, swing_clearance,
                               landing_position):
    """Get the tracking target for the toes during the swing phase.

    The swing toe will move 70% of the distance in the first half of the swing.
    Intuitely, we want to move the swing foot quickly to the target landing
    location and stay above the ground, in this way the control is more robust
    to perturbations to the body that may cause the swing foot to drop onto
    the ground earlier than expected. This is a common practice similar
    to the MIT cheetah and Marc Raibert's original controllers. After the
    designated swing motion finishes, we also command the foot to go down
    for a short ditance (0.03m). This is to mitigate issues when the swing
    finishes before it touches the ground.

    Args:
      lift_off_position: Local position when the foot leaves ground.
      phase: Normalized phase of the foot in the current swing cycle.
      swing_clearance: Height of the highest point in the swing trajectory.
      landing_position: Target landing position in the local space.

    Returns:
      The interpolated foot target for the current step.
    """
    # Up vector in the world coordinate (without considering yaw).
    rotated_up_vec = np.array(
        self.pybullet_client.multiplyTransforms(
            (0, 0, 0),
            self.pybullet_client.getQuaternionFromEuler(
                (self.robot.base_roll_pitch_yaw[0],
                 self.robot.base_roll_pitch_yaw[1], 0)), (0, 0, 1),
            _UNIT_QUATERNION)[0])

    # Linearly interpolate the trajectory to get the foot target.
    keyframe_timings = [0.0, 0.45, 0.9, 0.9001, 1.0]
    peak_toe_position = 0.3 * lift_off_position + 0.7 * landing_position + rotated_up_vec * swing_clearance
    # TODO(magicmelon): Update the gait generator to warp the gait and keep
    # the swing leg going down until it touches the ground.
    landing_position_pressing_down = landing_position - rotated_up_vec * 0.03
    keyframe_positions = np.array([
        lift_off_position, peak_toe_position, landing_position,
        landing_position_pressing_down, landing_position_pressing_down
    ])

    target_toe_positions = np.array([
        np.interp(phase, keyframe_timings, keyframe_positions[:, 0]),
        np.interp(phase, keyframe_timings, keyframe_positions[:, 1]),
        np.interp(phase, keyframe_timings, keyframe_positions[:, 2])
    ])
    return target_toe_positions

  def step(self, action: Sequence[float]):
    """Steps the wrapped environment.

    Args:
      action:

    Returns:
      The tuple containing the observation, the reward, and the episode
        end indicator.
    """
    self.last_action = self._clean_up_action(action)

    obs, reward, done, info = self._step_motion_controller(self.last_action)
    self._step_counter += 1
    self._fill_observations(obs)
    self._observation = obs

    return obs, reward, done, info

  def _extract_action(self, action, name):
    return action[self.
                  _action_start_indices[name]:self._action_start_indices[name] +
                  self._action_dimensions[name]]

  def _get_swing_foot_ids(self):
    swing_foot_ids = []
    for leg_id in range(_NUM_LEGS):
      if self._gait_generator.leg_state[
          leg_id] == gait_generator_lib.LegState.SWING:
        swing_foot_ids.append(leg_id)
    return swing_foot_ids

  def _update_gait_states_and_flags(self):
    """Update gait-related variables and flags."""
    current_leg_state = self._gait_generator.leg_state
    for leg_id in range(_NUM_LEGS):
      if current_leg_state[
          leg_id] == gait_generator_lib.LegState.SWING and self._last_leg_state[
              leg_id] != gait_generator_lib.LegState.SWING:
        self._lift_off_positions[leg_id] = self._gym_env.robot.foot_positions(
        )[leg_id]
        self._need_new_swing_target[leg_id] = 1
    self._last_leg_state = copy.copy(current_leg_state)

    com_estimate_leg_indices = []
    for leg_id in range(_NUM_LEGS):
      # Use the ones not swinging to estimate the com height
      if leg_id not in self._get_swing_foot_ids():
        com_estimate_leg_indices.append(leg_id)
    self._com_height_estimator.com_estimate_leg_indices = com_estimate_leg_indices
    self._state_estimator.update(self.robot.GetTimeSinceReset() -
                                 self._mpc_reset_time)
    self._gait_generator.update(self.robot.GetTimeSinceReset() -
                                self._mpc_reset_time)

  def _process_action(self, action):
    """Process the action and set relevant variables."""
    # Extract the swing targets from the input action.
    for leg_id in range(_NUM_LEGS):
      if self._need_new_swing_target[leg_id]:
        self._nominal_swing_leg_commands[leg_id]["swing_target"] = (
            self._extract_action(action, "swing_targets_" + str(leg_id)))
        self._nominal_swing_leg_commands[leg_id]["swing_clearance"] = (
            self._extract_action(action, "swing_clearance_" + str(leg_id)))

        self._actual_swing_leg_commands[leg_id]["swing_target"] = np.copy(
            self._nominal_swing_leg_commands[leg_id]["swing_target"])
        self._actual_swing_leg_commands[leg_id][
            "swing_clearance"] = self._nominal_swing_leg_commands[leg_id][
                "swing_clearance"]

    # Reset the flags so the next high level commands are not used until the
    # next swing happens.
    self._need_new_swing_target = np.zeros(_NUM_LEGS)

    # Extract the delta swing targets from the input action.
    if self._policy_output_swing_action_delta:
      for leg_id in self._get_swing_foot_ids():
        self._actual_swing_leg_commands[leg_id]["swing_target"] = (
            self._nominal_swing_leg_commands[leg_id]["swing_target"] +
            self._extract_action(action, "swing_targets_delta_" + str(leg_id)))
        self._actual_swing_leg_commands[leg_id]["swing_clearance"] = (
            self._nominal_swing_leg_commands[leg_id]["swing_clearance"] +
            self._extract_action(action,
                                 "swing_clearance_delta_" + str(leg_id)))
    # Extract the target base movement commands from the input action.
    if "target_base_velocity" in self._action_names:
      self._target_base_velocity_from_policy = self._extract_action(
          action, "target_base_velocity")
    else:
      self._target_base_velocity_from_policy = np.zeros(3)
    desired_twist_speed = self._extract_action(action, "base_twist")
    desired_body_height = self._extract_action(action, "base_height")
    desired_roll_pitch = self._extract_action(action, "base_roll_pitch")

    self._stance_controller.desired_twisting_speed = desired_twist_speed
    self._stance_controller.desired_body_height = desired_body_height
    self._stance_controller.desired_roll_pitch = desired_roll_pitch

  def _compute_swing_action(self):
    """Compute actions for the swing legs."""
    local_toe_positions = np.array(
        self._gym_env.robot.foot_positions(position_in_world_frame=False))
    toe_positions_over_time = copy.deepcopy(local_toe_positions)

    for leg_id in self._get_swing_foot_ids():
      target_toe_positions_local = self._get_toe_tracking_target(
          self._lift_off_positions[leg_id],
          self._gait_generator.normalized_phase[leg_id],
          self._actual_swing_leg_commands[leg_id]["swing_clearance"],
          self._actual_swing_leg_commands[leg_id]["swing_target"] +
          self._initial_local_toe_positions[leg_id])
      toe_positions_over_time[leg_id] = target_toe_positions_local
    self._current_toe_target = np.reshape(
        copy.deepcopy(toe_positions_over_time), 3 * _NUM_LEGS)
    return np.array(
        self.robot.motor_angles_from_foot_positions(
            toe_positions_over_time, position_in_world_frame=False)[1])

  def _compute_stance_action(self):
    """Compute actions for the stance legs."""

    # update target com velocity by combining policy output and heuristics
    hip_relative_swing_targets = []
    swing_durations = []
    com_horizontal_velocity = np.array(
        self._state_estimator.com_velocity_body_yaw_aligned_frame)[0:2]
    for swing_id in self._get_swing_foot_ids():
      hip_relative_swing_targets.append(
          np.array([
              self._actual_swing_leg_commands[swing_id]["swing_target"][0],
              self._actual_swing_leg_commands[swing_id]["swing_target"][1]
          ]))
      swing_durations.append(self._gait_generator.swing_duration[swing_id])

    self._target_horizontal_com_velocity_heuristic.update_horizontal_com_velocity_heuristic(
        hip_relative_swing_targets, com_horizontal_velocity, swing_durations)
    self._stance_controller.desired_speed = np.array(
        self._target_base_velocity_from_policy)
    self._stance_controller.desired_speed[
        0:
        2] += self._target_horizontal_com_velocity_heuristic.horizontal_com_velocity_heuristic

    # compute actions for the stance leg
    return self._stance_controller.get_action()

  def _combine_swing_stance_action(self, swing_action, stance_action):
    """Combine stance and swing leg actions."""
    feet_contact_states = copy.copy(self._gait_generator.leg_state)
    actions = []
    for leg_id in range(_NUM_LEGS):
      if leg_id in self._get_swing_foot_ids(
      ) and feet_contact_states[leg_id] == gait_generator_lib.LegState.SWING:
        for motor_id_in_leg in range(_MOTORS_PER_LEG):
          actions.extend(
              (swing_action[leg_id * _MOTORS_PER_LEG + motor_id_in_leg],
               _MOTOR_KP[leg_id * _MOTORS_PER_LEG + motor_id_in_leg], 0.0,
               _MOTOR_KD[leg_id * _MOTORS_PER_LEG + motor_id_in_leg], 0.0))
      else:
        for motor_id_in_leg in range(_MOTORS_PER_LEG):
          actions.extend(stance_action[leg_id * _MOTORS_PER_LEG +
                                       motor_id_in_leg])
    return actions

  def _step_motion_controller(self, action):
    """Run the MPC controller to advance the robot's state."""

    self._process_action(action)

    # run MPC-based control for a certain amount of time
    total_reward = 0.0
    start_time = self._gym_env.robot.GetTimeSinceReset()
    while self._gym_env.robot.GetTimeSinceReset(
    ) - start_time < self._time_per_control_step:
      self._update_gait_states_and_flags()

      swing_action = self._compute_swing_action()

      stance_action = self._compute_stance_action()

      actions = self._combine_swing_stance_action(swing_action, stance_action)

      obs, rew, done, info = self._gym_env.step(actions)
      if self._stance_controller.qp_solver_fail:
        done = True
      total_reward += rew

      if done:
        return obs, 0.0, done, info

    return obs, total_reward, done, info

  @property
  def observation(self):
    return self._observation

  @property
  def env_step_counter(self):
    return self._step_counter
