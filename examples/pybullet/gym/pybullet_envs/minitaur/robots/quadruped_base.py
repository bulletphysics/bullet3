# Lint as: python3
"""The base class for all quadrupeds."""
from typing import Any, Callable, Dict, Sequence, Tuple, Text, Union
import gin
import gym
import numpy as np

from pybullet_utils import bullet_client
from pybullet_envs.minitaur.envs_v2.sensors import sensor as sensor_lib
from pybullet_envs.minitaur.robots import hybrid_motor_model
from pybullet_envs.minitaur.robots import robot_base
from pybullet_envs.minitaur.robots import robot_config
from pybullet_envs.minitaur.robots import robot_urdf_loader
from pybullet_envs.minitaur.robots.safety import data_types as safety_data_types
from pybullet_envs.minitaur.robots.utilities import kinematics_utils

_UNIT_QUATERNION = (0, 0, 0, 1)
_GRAVITY_ACCELERATION_OFFSET = (0, 0, 10)


@gin.configurable
class QuadrupedBase(robot_base.RobotBase):
  """The basic quadruped class for both sim and real robots."""

  def __init__(
      self,
      pybullet_client: bullet_client.BulletClient,
      clock: Callable[..., float],
      motor_control_mode: robot_config.MotorControlMode,
      motor_limits: robot_config.MotorLimits,
      motor_model_class: Any = hybrid_motor_model.HybridMotorModel,
      action_filter: Any = None,
      sensors: Sequence[sensor_lib.Sensor] = (),
      safety_config: safety_data_types.SafetyConfig = None,
      **kwargs,
  ):
    """Initializes the class.

    Args:
      pybullet_client: The PyBullet client.
      clock: The sim or real clock. The clock function is typically provided by
        the gym environment.
      motor_control_mode: Specifies in which mode the motor operates.
      motor_limits: The motor limits of the robot. Used by the motor_model_class
        and action space building.
      motor_model_class: The motor model to use. Not needed for real robots.
      action_filter: The filter to smooth and/or regulate the actions.
      sensors: All sensors mounted on the robot.
      safety_config: The safety setting for the robot.
      **kwargs: Additional args.
    """

    self._pybullet_client = pybullet_client
    self._clock = clock
    self._motor_control_mode = motor_control_mode
    self._motor_model_class = motor_model_class
    self._motor_limits = motor_limits
    self._action_space = None
    self._action_names = None
    self._action_filter = action_filter
    self._sensors = sensors
    self._safety_config = safety_config
    self._urdf_loader = None
    self._last_base_velocity = np.zeros(3)
    self._last_observation_time = self._clock()
    self._last_base_acceleration_world = np.zeros(3)
    self._last_base_acceleration_accelerometer = np.zeros(3)

    self.load()

  def load(
      self,
      base_position: Tuple[float] = None,
      base_orientation_quaternion: Tuple[float] = None,
      joint_angles: Union[Dict[Text, float], Tuple[float]] = None,
  ):
    """Loads the URDF with the configured pose.

    Args:
      base_position: The base position after URDF loading. Will use the
        configured pose in gin if None.
      base_orientation_quaternion: The base orientation after URDF loading. Will
        use the configured values in gin if not specified.
      joint_angles: The desired joint angles after loading. Will use the
        configured values if None.
    """
    # A robot specific pre loading routing.
    self._pre_load()

    if not self._urdf_loader:
      self._urdf_loader = robot_urdf_loader.RobotUrdfLoader(
          pybullet_client=self._pybullet_client)

    # Record the urdf pose at loading, which will be used as the rotation
    # reference for base rotation computation.
    self._init_urdf_position, self._init_orientation_quat = (
        self._pybullet_client.getBasePositionAndOrientation(
            self._urdf_loader.robot_id))
    unused_position, self._init_orientation_inv_quat = (
        self._pybullet_client.invertTransform(
            position=(0, 0, 0), orientation=self._init_orientation_quat))

    # Joint ids may be different from the motor ids.
    self._joint_id_dict = self._urdf_loader.get_joint_id_dict()
    for joint_id in self._joint_id_dict.values():
      # Disables the default motors in PyBullet.
      self._pybullet_client.setJointMotorControl2(
          bodyIndex=self._urdf_loader.robot_id,
          jointIndex=joint_id,
          controlMode=self._pybullet_client.VELOCITY_CONTROL,
          targetVelocity=0,
          force=0)
      # Removes the default joint damping in PyBullet.
      self._pybullet_client.changeDynamics(
          self._urdf_loader.robot_id,
          joint_id,
          linearDamping=0,
          angularDamping=0)

    # We expect that this is non-empty for all quadrupedes, and should be an
    # OrderedDict.
    self._motor_id_dict = self._urdf_loader.get_motor_id_dict()
    if not self._motor_id_dict:
      raise ValueError("Motor id dict cannot be empty for quadrupeds.")
    self._motor_ids = self._motor_id_dict.values()
    self._num_motors = len(self._motor_id_dict)

    self._build_action_space()

    # Not needed for real robots.
    if self._motor_model_class:
      # TODO(b/151664871): Also supports position/velocity limits in the motor
      # model.
      self._motor_model = self._motor_model_class(
          num_motors=self._num_motors,
          motor_control_mode=self._motor_control_mode,
          torque_lower_limits=self._motor_limits.torque_lower_limits,
          torque_upper_limits=self._motor_limits.torque_upper_limits,
      )

    # Caches the variable for faster computation during stepping.
    self._motor_direction_dict = self._urdf_loader.get_joint_direction_dict(
        self._motor_id_dict.keys())
    self._motor_directions = np.array(list(self._motor_direction_dict.values()))

    self._motor_offset_dict = self._urdf_loader.get_joint_offset_dict(
        self._motor_id_dict.keys())
    self._motor_offsets = np.array(list(self._motor_offset_dict.values()))

    # A robot specific routine post loading.
    self._on_load()

    # Robot sensors may use information from the class. So we initialize them
    # after the loading is done.
    for sensor in self._sensors:
      sensor.set_robot(self)

  def _build_action_space(self):
    """Builds the action space of the robot using the motor limits."""
    if self._motor_control_mode == robot_config.MotorControlMode.POSITION:
      self._action_space = gym.spaces.Box(
          low=self._motor_limits.angle_lower_limits,
          high=self._motor_limits.angle_upper_limits,
          shape=(self._num_motors,),
          dtype=np.float32)  # TODO(b/159160184) Make dtype configurable.
      self._action_names = tuple(
          "POSITION_{}".format(motor) for motor in self._motor_id_dict.keys())
    elif self._motor_control_mode == robot_config.MotorControlMode.TORQUE:
      self._action_space = gym.spaces.Box(
          low=self._motor_limits.torque_lower_limits,
          high=self._motor_limits.torque_upper_limits,
          shape=(self._num_motors,),
          dtype=np.float32)
      self._action_names = tuple(
          "TORQUE_{}".format(motor) for motor in self._motor_id_dict.keys())
    elif self._motor_control_mode == robot_config.MotorControlMode.HYBRID:
      hybrid_action_limits_low = [
          self._motor_limits.angle_lower_limits,  # q
          # q_dot
          self._motor_limits.velocity_lower_limits,
          0,  # kp
          0,  # kd
          self._motor_limits.torque_lower_limits
      ]  # tau
      hybrid_action_limits_high = [
          self._motor_limits.angle_upper_limits,
          self._motor_limits.velocity_upper_limits, np.inf, np.inf,
          self._motor_limits.torque_upper_limits
      ]
      space_low = np.full(
          (self._num_motors, robot_config.HYBRID_ACTION_DIMENSION),
          hybrid_action_limits_low).ravel()
      space_high = np.full(
          (self._num_motors, robot_config.HYBRID_ACTION_DIMENSION),
          hybrid_action_limits_high).ravel()
      self._action_space = gym.spaces.Box(
          low=space_low, high=space_high, dtype=np.float32)
      self._action_names = tuple(
          "HYBRID_{}".format(motor) for motor in self._motor_id_dict.keys())
    else:
      raise NotImplementedError("Not yet implemented!")

  def _pre_load(self):
    """Robot specific pre load routine.

    For example, this allows configuration of the URDF loader.
    """
    pass

  def _on_load(self):
    """Robot specific post load routine.

    For example, we need to add add additional hinge constraints to the leg
    components of Minitaur after loading.

    """
    pass

  @gin.configurable
  def reset(
      self,
      base_position: Tuple[float] = None,
      base_orientation_quaternion: Tuple[float] = None,
      joint_angles: Union[Dict[Text, float], Tuple[float]] = None,
      save_base_pose: bool = False,
      **kwargs,
  ):
    """Resets the robot base and joint pose without reloading the URDF.

    Base pose resetting only works for simulated robots or visualization of real
    robots. This routine also updates the initial observation dict.

    Args:
      base_position: The desired base position. Will use the configured pose in
        gin if None. Does not affect the position of the real robots in general.
      base_orientation_quaternion: The base orientation after resetting. Will
        use the configured values in gin if not specified.
      joint_angles: The desired joint angles after resetting. Will use the
        configured values if None.
      save_base_pose: Save the base position and orientation as the default pose
        after resetting.
      **kwargs: Other args for backward compatibility. TODO(b/151975607): Remove
        after migration.
    """
    # Reset the robot's motor model.
    self._motor_model.reset()

    # Reset the quantities for computing base acceleration.
    self._last_base_velocity = np.zeros(3)
    self._last_observation_time = self._clock()
    self._last_base_acceleration_world = np.zeros(3)
    self._last_base_acceleration_accelerometer = np.zeros(3)

    # Solves chicken and egg problem. We need to run a control step to obtain
    # the first motor torques.
    self._motor_torques = np.zeros(self._num_motors)

    # Receives a set of observation from the robot in case the reset function
    # needs to use them.
    self.receive_observation()

    self._reset_base_pose(base_position, base_orientation_quaternion)
    self._reset_joint_angles(joint_angles)

    if save_base_pose:
      # Records the base pose at resetting again, in case Reset is called with a
      # different base orientation. This base pose will be used as zero
      # rotation reference for base rotation computation.
      self._init_urdf_position, self._init_orientation_quat = (
          self._pybullet_client.getBasePositionAndOrientation(
              self._urdf_loader.robot_id))
      unused_position, self._init_orientation_inv_quat = (
          self._pybullet_client.invertTransform(
              position=(0, 0, 0), orientation=self._init_orientation_quat))

    # Updates the observation at the end of resetting.
    self.receive_observation()
    self._time_at_reset = self._clock()

  def GetTimeSinceReset(self):
    return self._clock() - self._time_at_reset

  def _reset_base_pose(self, position=None, orientation_quat=None):
    """Resets the pose of the robot's base.

    Base pose resetting only works for simulated robots or visualization of real
    robots.

    Args:
      position: The desired base position. Will use the configured pose in gin
        if None.
      orientation_quat: The desired base rotation. Will use the configured
        default pose in None.
    """
    self._urdf_loader.reset_base_pose(position, orientation_quat)

  def _reset_joint_angles(self,
                          joint_angles: Union[Tuple[float],
                                              Dict[Text, float]] = None):
    """Resets the joint pose.

    Real robots need to specify their routine to send joint angles. Simulated
    Minitaur robots also needs to use dynamics to drive the motor joints, due to
    the additional hinge joints not present in the URDF.

    Args:
      joint_angles: The joint pose if provided. Will use the robot default pose
        from configuration.
    """
    # TODO(b/148897311): Supports tuple as the input.
    self._urdf_loader.reset_joint_angles(joint_angles)

  def terminate(self):
    """The safe exit routine for the robot.

    Only implemented for real robots.

    """
    pass

  def step(self, action: Any, num_sub_steps: int = 1):
    """Steps the simulation.

    This is maintained for backward compatibility with the old robot class.

    Args:
      action: The control command to be executed by the robot.
      num_sub_steps: Each action can be applied (possibly with interpolation)
        multiple timesteps, to simulate the elapsed time between two consecutive
        commands on real robots.
    """
    action = self.pre_control_step(action)

    for _ in range(num_sub_steps):
      # TODO(b/149252003): Add sub sampling.
      self.apply_action(action)
      # Timestep is pre-determined at simulation setup.
      self._pybullet_client.stepSimulation()
      self.receive_observation()

    self.post_control_step()

  def pre_control_step(self, action: Any, control_timestep: float = None):
    """Processes the action and updates per control step quantities.

    Args:
      action: The input control command.
      control_timestep: The control time step in the environment.
        TODO(b/153835005), we can remove this once we pass env to the robot.

    Returns:
      The filtered action.
    """
    if self._action_filter:
      # We assume the filter will create a set of interpolated results.
      action = self._action_filter.filter(action)
    return action

  def apply_action(self, motor_commands, motor_control_mode=None):

    # TODO(b/148897311): Supports dict in the future.
    motor_commands = np.asarray(motor_commands)

    # We always use torque based control at the lowest level for quadrupeds.
    unused_observed_torques, actual_torques = (
        self._motor_model.get_motor_torques(motor_commands, motor_control_mode))
    self._motor_torques = actual_torques

    # Converts the motor torques to URDF joint space, which may have different
    # directions.
    applied_motor_torques = np.multiply(actual_torques, self._motor_directions)

    self._pybullet_client.setJointMotorControlArray(
        bodyIndex=self._urdf_loader.robot_id,
        jointIndices=self._motor_ids,
        controlMode=self._pybullet_client.TORQUE_CONTROL,
        forces=applied_motor_torques)

  def _get_base_roll_pitch_yaw_rate(self):
    _, angular_velocity = self._pybullet_client.getBaseVelocity(
        self._urdf_loader.robot_id)
    return kinematics_utils.rotate_to_base_frame(
        self._pybullet_client, self.urdf_loader.robot_id, angular_velocity,
        self._init_orientation_inv_quat)

  def _get_base_velocity(self):
    base_velocity, _ = self._pybullet_client.getBaseVelocity(
        self._urdf_loader.robot_id)
    return base_velocity

  def _update_base_acceleration(self):
    """Update the base acceleration using finite difference."""
    if self._last_observation_time < self.timestamp:
      self._last_base_acceleration_world = (
          np.array(self._base_velocity) - self._last_base_velocity) / (
              self.timestamp - self._last_observation_time)
      _, inv_base_orientation = self.pybullet_client.invertTransform(
          np.zeros(3), np.array(self.base_orientation_quaternion))

      # An offset is added to the acceleration measured in the world frame
      # because the accelerometer reading is in the frame of free-falling robot.
      base_acceleration_accelerometer = self.pybullet_client.multiplyTransforms(
          np.zeros(3), inv_base_orientation,
          self._last_base_acceleration_world + _GRAVITY_ACCELERATION_OFFSET,
          _UNIT_QUATERNION)[0]
      self._last_base_acceleration_accelerometer = np.array(
          base_acceleration_accelerometer)

  def receive_observation(self):
    """Receives the observations for all sensors."""
    # Update the intrinsic values including the joint angles, joint
    # velocities, and imu readings.
    self._base_position, base_orientation_quat = (
        self._pybullet_client.getBasePositionAndOrientation(
            self._urdf_loader.robot_id))
    _, self._base_orientation_quat = self._pybullet_client.multiplyTransforms(
        positionA=(0, 0, 0),
        orientationA=self._init_orientation_inv_quat,
        positionB=(0, 0, 0),
        orientationB=base_orientation_quat)
    self._base_velocity = self._get_base_velocity()
    self._base_roll_pitch_yaw = self._pybullet_client.getEulerFromQuaternion(
        self._base_orientation_quat)

    self._base_roll_pitch_yaw_rate = self._get_base_roll_pitch_yaw_rate()

    self._joint_states = self._pybullet_client.getJointStates(
        self._urdf_loader.robot_id, self._motor_ids)
    self._motor_angles = np.array(
        [joint_state[0] for joint_state in self._joint_states])
    self._motor_angles = (self._motor_angles -
                          self._motor_offsets) * self._motor_directions

    self._motor_velocities = np.array(
        [joint_state[1] for joint_state in self._joint_states])
    self._motor_velocities = self._motor_velocities * self._motor_directions

    # We use motor models to track the delayed motor positions and velocities
    # buffer.
    if self._motor_model:
      self._motor_model.update(self._clock(), self._motor_angles,
                               self._motor_velocities)

    self._update_base_acceleration()
    # Update the latest base velocity and timestamp at the end of the API.
    self._last_base_velocity = np.array(self._base_velocity)
    self._last_observation_time = self.timestamp

  def post_control_step(self):
    """Called at the end of a control step outside the action repeat loop."""
    pass

  # TODO(tingnan): Change from "foot_positions" to "feet_positions".
  def motor_angles_from_foot_positions(self,
                                       foot_positions,
                                       position_in_world_frame=False):
    """Use IK to compute the motor angles, given the feet links' positions.

    Args:
      foot_positions: The foot links' positions in frame specified by the next
        parameter. The input is a numpy array of size (4, 3).
      position_in_world_frame: Whether the foot_positions are specified in the
        world frame.

    Returns:
      A tuple. The position indices and the angles for all joints along the
      leg. The position indices is consistent with the joint orders as returned
      by GetMotorAngles API.
    """
    joint_position_idxs = np.arange(self.num_motors)
    foot_link_ids = tuple(self._urdf_loader.get_end_effector_id_dict().values())
    joint_angles = kinematics_utils.joint_angles_from_link_positions(
        pybullet_client=self.pybullet_client,
        urdf_id=self.robot_id,
        link_positions=foot_positions,
        link_ids=foot_link_ids,
        joint_dof_ids=joint_position_idxs,
        positions_are_in_world_frame=position_in_world_frame)
    joint_angles = np.multiply(
        np.asarray(joint_angles) - np.asarray(self._motor_offsets),
        self._motor_directions)
    return joint_position_idxs, joint_angles

  # TODO(tingnan): Change from "foot_positions" to "feet_positions".
  def foot_positions(self, position_in_world_frame=False):
    """Returns the robot's foot positions in the base/world frame."""
    foot_positions = []
    foot_link_ids = tuple(self._urdf_loader.get_end_effector_id_dict().values())
    for foot_id in foot_link_ids:
      if not position_in_world_frame:
        foot_positions.append(
            kinematics_utils.link_position_in_base_frame(
                pybullet_client=self.pybullet_client,
                urdf_id=self.robot_id,
                link_id=foot_id,
            ))
      else:
        foot_positions.append(
            kinematics_utils.link_position_in_world_frame(
                pybullet_client=self.pybullet_client,
                urdf_id=self.robot_id,
                link_id=foot_id,
            ))
    return np.array(foot_positions)

  def feet_contact_forces(self) -> Sequence[np.ndarray]:
    """Gets the contact forces on all feet.

    Reals robot may use a robot specific implementation. For example, the
    Laikago will measure each contact force in the corresponding foot's local
    frame, and this force will not be the total contact force due to the sensor
    limitation.

    For simulated robots, we wll always report the force in the base frame.

    Returns:
      A list of foot contact forces.
    """
    foot_link_ids = tuple(self._urdf_loader.get_end_effector_id_dict().values())
    contact_forces = [np.zeros(3) for _ in range(len(foot_link_ids))]

    all_contacts = self._pybullet_client.getContactPoints(
        bodyA=self._urdf_loader.robot_id)

    for contact in all_contacts:
      (unused_flag, body_a_id, body_b_id, link_a_id, unused_link_b_id,
       unused_pos_on_a, unused_pos_on_b, contact_normal_b_to_a, unused_distance,
       normal_force, friction_1, friction_direction_1, friction_2,
       friction_direction_2) = contact

      # Ignore self contacts
      if body_b_id == body_a_id:
        continue

      if link_a_id in foot_link_ids:
        normal_force = np.array(contact_normal_b_to_a) * normal_force
        friction_force = np.array(friction_direction_1) * friction_1 + np.array(
            friction_direction_2) * friction_2
        force = normal_force + friction_force
        local_force = kinematics_utils.rotate_to_base_frame(
            self._pybullet_client, self.urdf_loader.robot_id, force,
            self._init_orientation_inv_quat)
        local_force_norm = np.linalg.norm(local_force)
        toe_link_order = foot_link_ids.index(link_a_id)
        if local_force_norm > 0:
          contact_forces[toe_link_order] += local_force
      else:
        continue
    return contact_forces

  def compute_jacobian_for_one_leg(self, leg_id: int) -> np.ndarray:
    """Compute the Jacobian for a given leg.

    Args:
      leg_id: Index of the leg for which the jacobian is computed.

    Returns:
      The 3 x N transposed Jacobian matrix. where N is the total DoFs of the
      robot. For a quadruped, the first 6 columns of the matrix corresponds to
      the CoM translation and rotation. The columns corresponds to a leg can be
      extracted with indices [6 + leg_id * 3: 6 + leg_id * 3 + 3]. Note that
      the jacobian is calculated for motors, which takes motor directions into
      consideration.
    """
    com_dof = self._urdf_loader.com_dof
    foot_link_ids = tuple(self._urdf_loader.get_end_effector_id_dict().values())
    return kinematics_utils.compute_jacobian(
        pybullet_client=self.pybullet_client,
        urdf_id=self.robot_id,
        link_id=foot_link_ids[leg_id],
        all_joint_positions=[
            state[0] for state in self._joint_states
        ]) * np.concatenate([np.ones(com_dof), self._motor_directions])

  def map_contact_force_to_joint_torques(
      self, leg_id: int, contact_force: np.ndarray) -> Dict[int, float]:
    """Maps the foot contact force to the leg joint torques.

    Args:
      leg_id: Index of the leg for which the jacobian is computed.
      contact_force: Desired contact force experted by the leg.

    Returns:
      A dict containing the torques for each motor on the leg.
    """
    foot_link_ids = tuple(self._urdf_loader.get_end_effector_id_dict().values())
    jv = self.compute_jacobian_for_one_leg(leg_id)
    all_motor_torques = np.matmul(contact_force, jv)
    motor_torques = {}
    motors_per_leg = self.num_motors // len(foot_link_ids)
    com_dof = self._urdf_loader.com_dof
    for joint_id in range(leg_id * motors_per_leg,
                          (leg_id + 1) * motors_per_leg):
      motor_torques[joint_id] = all_motor_torques[com_dof + joint_id]

    return motor_torques

  @classmethod
  def get_constants(cls):
    raise NotImplementedError("Not yet implemented!")

  @property
  def timestamp(self):
    return self._clock()

  @property
  def action_space(self):
    return self._action_space

  @property
  def action_names(self):
    return self._action_names

  @property
  def base_orientation_quaternion(self):
    """Gets the base orientation as a quaternion.

    The base orientation is always relative to the init_orientation, which
    can be updated by Reset function. This is necessary as many URDF can have an
    internal frame that is not z-up, so if we don't provide an init_orientation
    (through Reset), the loaded robot can have its belly facing the horizontal
    direction.

    Returns:
      The base orientation in quaternion.
    """
    return self._base_orientation_quat

  @property
  def base_orientation_quaternion_default_frame(self):
    """Gets the base orientation in the robot's default frame.

    This is the base orientation in whatever frame the robot specifies. For
    simulated robot this is the URDF's internal frame. For real robot this can
    be based on the rpy reading determined by the IMU.

    Returns:
      The base orientation in quaternion in a robot default frame.
    """
    _, base_orientation_quat = (
        self._pybullet_client.getBasePositionAndOrientation(
            self._urdf_loader.robot_id))
    return base_orientation_quat

  @property
  def sensors(self):
    return self._sensors

  @property
  def base_roll_pitch_yaw(self):
    return self._base_roll_pitch_yaw

  @property
  def base_roll_pitch_yaw_rate(self):
    return self._base_roll_pitch_yaw_rate

  @property
  def base_position(self):
    return self._base_position

  @property
  def base_velocity(self):
    return self._base_velocity

  @property
  def is_safe(self):
    return True

  @property
  def num_motors(self):
    return self._num_motors

  @property
  def motor_model(self):
    return self._motor_model

  @property
  def motor_limits(self) -> robot_config.MotorLimits:
    return self._motor_limits

  @property
  def motor_angles(self):
    return self._motor_angles

  @property
  def motor_velocities(self):
    return self._motor_velocities

  @property
  def motor_torques(self):
    return self._motor_torques

  @property
  def pybullet_client(self):
    return self._pybullet_client

  @property
  def urdf_loader(self):
    return self._urdf_loader

  @property
  def robot_id(self):
    return self._urdf_loader.robot_id

  @property
  def initital_orientation_inverse_quaternion(self):
    return self._init_orientation_inv_quat

  @property
  def base_acceleration_accelerometer(self):
    """Get the base acceleration measured by an accelerometer.

    The acceleration is measured in the local frame of a free-falling robot,
    which is consistent with the robot's IMU measurements. Here the
    gravitational acceleration is first added to the acceleration in the world
    frame, which is then converted to the local frame of the robot.

    """
    return np.array(self._last_base_acceleration_accelerometer)

  @property
  def base_acceleration(self):
    """Get the base acceleration in the world frame."""
    return np.array(self._last_base_acceleration_world)
