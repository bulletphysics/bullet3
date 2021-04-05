"""Change the action from uv of the depth map to xyz in world frame."""

import copy
import math
from typing import Sequence
import gin
import gym
import numpy as np

from pybullet_envs.minitaur.robots import laikago_kinematic_constants
from pybullet_envs.minitaur.robots import quadruped_base
from pybullet_envs.minitaur.robots import wheeled_robot_base_sim

UNIT_QUATERNION = (0, 0, 0, 1)
# A small gap between the target foot position and the floor.
FLOOR_HEIGHT_EPSILON = 0.02
FOOT_HEIGHT_CLEARNCE = 0.075
BASE_MOVEMENT_SPEED = 0.3
INIT_ABDUCTION_ANGLE = laikago_kinematic_constants.INIT_ABDUCTION_ANGLE
INIT_HIP_ANGLE = laikago_kinematic_constants.INIT_HIP_ANGLE
INIT_KNEE_ANGLE = laikago_kinematic_constants.INIT_KNEE_ANGLE
NUM_LEGS = laikago_kinematic_constants.NUM_LEGS
COM_VIZ_BOX_SIZE = [0.025, 0.025, 0.005]
_DEFAULT_JOINT_POSE = (INIT_ABDUCTION_ANGLE, INIT_HIP_ANGLE,
                       INIT_KNEE_ANGLE) * NUM_LEGS
# Weights for computing the target COM from the supporting feet locations.
# The target COM for the front feet are biased forward for better robot
# stability and seeing farther in distance, which enables larger steps.
_SUPPORT_WEIGHT_MAP = [[0.0, 0.4, 0.3, 0.3], [0.4, 0.0, 0.3, 0.3],
                       [1.0 / 3, 1.0 / 3, 0.0, 1.0 / 3],
                       [1.0 / 3, 1.0 / 3, 1.0 / 3, 0.0]]
_TASK_SENSOR_NAME = "sensors"


@gin.configurable
class DepthUVToFootPlacementWrapper(object):
  """Changes the action from a point in depth map to a point in the world frame.

  Attributes:
    observation: The current observation of the environment.
    last_action: The last that was used to step the environment.
    env_step_counter: The number of control steps that have been elapesed since
      the environment is reset.
    action_space: The action space of the environment.
  """

  def __init__(self,
               gym_env,
               visualization=True,
               foot_movement=False,
               foot_clearance_height=FOOT_HEIGHT_CLEARNCE,
               base_movement_speed=BASE_MOVEMENT_SPEED,
               foothold_update_frequency=4):
    """Initializes the wrapper.

    Args:
      gym_env: the wrapped gym environment. The robot is controlled
        kinematically when gym_env.robot inherits from WheeledRobotBase and is
        controlled dynamically using a static gait controller when gym_env.robot
        inherits from QudrupedBase.
      visualization: whether to draw a sphere that represents the foothold
        position.
      foot_movement: whether move the toe to the desired foothold position using
        IK for visualization/debugging purpose.
      foot_clearance_height: the maximum height of the swing foot.
      base_movement_speed: the speed of the robot base.
      foothold_update_frequency: the frequency of updating the foothold, which
        is the same as the frequency of the steps. The default value 4 means
        four steps (1 complete cycle of static gait) per second.
    """
    self._gym_env = gym_env
    self._num_control_steps_per_foothold_update = max(
        1, int(1.0 / foothold_update_frequency / self._gym_env.env_time_step))
    self.last_action = None
    self._visualization = visualization
    self._foot_movement = foot_movement
    self._foot_clearance_height = foot_clearance_height
    self._base_movement_speed = base_movement_speed
    self._step_counter = 0
    self.observation_space.spaces["toe_position"] = gym.spaces.Box(
        np.array([-1.0] * 3), np.array([1.0] * 3))
    self.action_space = gym.spaces.Box(
        np.array([-1.0] * 2), np.array([1.0] * 2))
    self.task.reset(self)
    if hasattr(self.task, _TASK_SENSOR_NAME):
      self.observation_space.spaces[
          self.task.get_name()] = self.task.observation_space

  def __getattr__(self, attr):
    return getattr(self._gym_env, attr)

  def reset(self, **kwargs):
    """Reset the environment."""
    obs = self._gym_env.reset(**kwargs)
    self.task.reset(self)
    self._step_counter = 0
    current_end_effector_pos = np.array(
        self._gym_env.robot.foot_positions()[self.task.swing_foot_id])
    self.last_action = [0, 0]
    if self._visualization:
      self._create_foot_placement_visualization()
      self._create_com_visualization()
    self._initial_local_toe_positions = np.array(
        self._gym_env.robot.foot_positions(position_in_world_frame=False))

    # Move COM to prepare for the first swing step.
    if isinstance(self._gym_env.robot, quadruped_base.QuadrupedBase):
      obs, _ = self._move_com_dynamic(
          self.task.swing_foot_id,
          self._num_control_steps_per_foothold_update // 3 * 2)

    # TODO(b/157614175): Adds a toe_position_sensor.
    obs["toe_position"] = current_end_effector_pos
    obs["vision"] = self.task.get_depth_image_for_foot()
    obs["LastAction"] = self.last_action

    self._observation = obs
    return self._observation

  def _move_kinematic(self):
    """Move robot kinematically.

    Returns:
      The tuple containing the observation and env info.
    """
    if self._foot_movement:
      toe_positions = np.array(
          self._gym_env.robot.foot_positions(position_in_world_frame=True))
      toe_positions[:, 2] = FLOOR_HEIGHT_EPSILON
      destination_foothold_xyz_global = self.task.get_foothold_location(
          self.last_action, use_world_frame=True)
      destination_foothold_xyz_global[2] = FLOOR_HEIGHT_EPSILON
    joint_pose = _DEFAULT_JOINT_POSE
    for i in range(self._num_control_steps_per_foothold_update):
      if self._foot_movement:
        alpha = i / (self._num_control_steps_per_foothold_update - 1)
        toe_positions_over_time = copy.deepcopy(toe_positions)
        toe_positions_over_time[self.task.swing_foot_id] = (
            self._construct_foot_trajectories(
                alpha, toe_positions[self.task.swing_foot_id],
                destination_foothold_xyz_global, self._foot_clearance_height))
        joint_pose = np.array(
            self.robot.motor_angles_from_foot_positions(
                toe_positions_over_time, position_in_world_frame=True)[1])
      action = {
          wheeled_robot_base_sim.BASE_ACTION_NAME:
              (self._base_movement_speed, 0),
          wheeled_robot_base_sim.BODY_ACTION_NAME:
              joint_pose,
      }
      obs, _, _, info = self._gym_env.step(action)
    return obs, info

  def _move_com_dynamic(self, swing_foot_id, num_control_steps):
    """Move robot COM to the weightd center of the support polygon through dynamics.

    Args:
      swing_foot_id: Index of the swing foot.
      num_control_steps: Total number of control steps for moving the COM.

    Returns:
      The tuple containing the observation and env info.
    """

    toe_positions = np.array(
        self._gym_env.robot.foot_positions(position_in_world_frame=False))
    support_polygon_center = np.array([0.0, 0.0, 0.0])
    for i in range(NUM_LEGS):
      if i != swing_foot_id:
        support_polygon_center += toe_positions[i] * _SUPPORT_WEIGHT_MAP[
            swing_foot_id][i]
    support_polygon_center[2] = 0.0

    for i in range(num_control_steps):
      alpha = i / (num_control_steps - 1)
      # Create a flat phase towards the end to stabilize the com movement.
      alpha = np.clip(alpha * 1.1, 0, 1)
      toe_positions_over_time = copy.deepcopy(
          toe_positions) - alpha * support_polygon_center
      # Use initial toe height to maintain the overal base height.
      for j in range(len(toe_positions_over_time)):
        toe_positions_over_time[j][2] = self._initial_local_toe_positions[j][2]
      joint_pose = np.array(
          self.robot.motor_angles_from_foot_positions(
              toe_positions_over_time, position_in_world_frame=False)[1])
      obs, _, _, info = self._gym_env.step(joint_pose)
      self._update_com_visualization()
    return obs, info

  def _swing_leg_dynamic(self, swing_foot_id, destination_foothold_xyz_local,
                         num_control_steps):
    """Move swing leg to the target foothold through IK and dynamics.

    Args:
      swing_foot_id: Index of the swing foot.
      destination_foothold_xyz_local: Target foothold position.
      num_control_steps: Total number of control steps for swinging the leg.

    Returns:
      The tuple containing the observation and env info.
    """

    local_toe_positions = np.array(
        self._gym_env.robot.foot_positions(position_in_world_frame=False))

    # Move swing leg
    for i in range(num_control_steps):
      alpha = i / (num_control_steps - 1)
      toe_positions_over_time = copy.deepcopy(local_toe_positions)
      toe_positions_over_time[swing_foot_id] = (
          self._construct_foot_trajectories(alpha,
                                            local_toe_positions[swing_foot_id],
                                            destination_foothold_xyz_local,
                                            self._foot_clearance_height))
      joint_pose = np.array(
          self.robot.motor_angles_from_foot_positions(
              toe_positions_over_time, position_in_world_frame=False)[1])

      obs, _, _, info = self._gym_env.step(joint_pose)
    return obs, info

  def _move_dynamic(self):
    """Move robot dynamically.

    Returns:
      The tuple containing the observation and env info.
    """
    destination_foothold_xyz_local = self.task.get_foothold_location(
        self.last_action, use_world_frame=False)
    # Lift the target foothold slightly to account for thickness of the feet.
    destination_foothold_xyz_local[2] += FLOOR_HEIGHT_EPSILON

    # Swing leg in the first 1/3 of the duration.
    self._swing_leg_dynamic(self.task.swing_foot_id,
                            destination_foothold_xyz_local,
                            self._num_control_steps_per_foothold_update // 3)

    # Move COM in the rest 2/3 of the duration.
    obs, info = self._move_com_dynamic(
        self.task.next_swing_foot_id,
        self._num_control_steps_per_foothold_update // 3 * 2)

    return obs, info

  def step(self, action: Sequence[float]):
    """Steps the wrapped environment.

    Args:
      action: 2 dimensional numpy array between [-1.0, 1.0]. They represents the
        depth image pixel index. We assume that only one foot is swinging in
        this wrapper, and this is the foothold location for that swinging leg.
        The order of the swinging leg and the index of the current swinging leg
        is defined in stepstone_visiontask.py.

    Returns:
      The tuple containing the observation, the reward, and the episode
        end indicator.
    """
    self.last_action = action
    reward = self.task(self)
    done = self.task.done(self)
    if self._visualization:
      self._update_foothold_visualization(action)

    if isinstance(self._gym_env.robot, quadruped_base.QuadrupedBase):
      obs, info = self._move_dynamic()
    else:
      obs, info = self._move_kinematic()

    self._step_counter += 1
    current_end_effector_pos = np.array(
        self._gym_env.robot.foot_positions()[self.task.swing_foot_id])
    obs["toe_position"] = current_end_effector_pos
    obs["vision"] = self.task.get_depth_image_for_foot()
    self._observation = obs
    return obs, reward, done, info

  def _construct_foot_trajectories(self, alpha, swing_foot_start_position,
                                   swing_foot_destination,
                                   foot_clearance_height):
    """Construct the target foot position for the swing foot.

    Args:
      alpha: a float in [0.0, 1.0] indicating the phase of the swing foot.
      swing_foot_start_position: foot position at the beginning of the swing
        motion.
      swing_foot_destination: target foot position at the end of the swing
        motion.
      foot_clearance_height: the maximum height of the swing foot.

    Returns:
      The interpolated swing foot position.
    """
    new_swing_foot_position = swing_foot_start_position + alpha * (
        swing_foot_destination - swing_foot_start_position)
    new_swing_foot_position[2] += (
        foot_clearance_height * math.sin((alpha) * math.pi))
    return new_swing_foot_position

  def _create_foot_placement_visualization(self):
    """Creates a visualization sphere that represents the foothold position."""
    visual_id = self._gym_env.pybullet_client.createVisualShape(
        self._gym_env.pybullet_client.GEOM_SPHERE,
        radius=0.02,
        rgbaColor=[0.7, 0.7, 0.7, 1])
    self._foothold_visual_body = self._gym_env.pybullet_client.createMultiBody(
        baseMass=0, baseVisualShapeIndex=visual_id, basePosition=[0, 0, 0])

  def _update_foothold_visualization(self, action):
    """Moves the visualization sphere that represents the next foothold."""
    foothold_xyz = self.task.get_foothold_location(
        action, use_world_frame=True)
    self._gym_env.pybullet_client.resetBasePositionAndOrientation(
        self._foothold_visual_body, foothold_xyz, UNIT_QUATERNION)

  def _create_com_visualization(self):
    """Creates visualization boxes for COM and support polygon center."""
    visual_id = self._gym_env.pybullet_client.createVisualShape(
        self._gym_env.pybullet_client.GEOM_BOX,
        halfExtents=COM_VIZ_BOX_SIZE,
        rgbaColor=[0.7, 0.5, 0.5, 1])
    self._support_polygon_center_visual_body = self._gym_env.pybullet_client.createMultiBody(
        baseMass=0, baseVisualShapeIndex=visual_id, basePosition=[0, 0, 0])

    visual_id = self._gym_env.pybullet_client.createVisualShape(
        self._gym_env.pybullet_client.GEOM_BOX,
        halfExtents=COM_VIZ_BOX_SIZE,
        rgbaColor=[0.5, 0.75, 0.5, 1])
    self._projected_com_visual_body = self._gym_env.pybullet_client.createMultiBody(
        baseMass=0, baseVisualShapeIndex=visual_id, basePosition=[0, 0, 0])

  def _update_com_visualization(self):
    """Moves the visualization for the COM and support polygon center."""
    toe_positions = np.array(
        self._gym_env.robot.foot_positions(position_in_world_frame=True))
    support_polygon_center_global = np.array([0.0, 0.0, 0.0])
    for i in range(NUM_LEGS):
      if i != self.task.next_swing_foot_id:
        support_polygon_center_global += toe_positions[i] * _SUPPORT_WEIGHT_MAP[
            self.task.next_swing_foot_id][i]
    support_polygon_center_global[2] = 0.0
    self._gym_env.pybullet_client.resetBasePositionAndOrientation(
        self._support_polygon_center_visual_body, support_polygon_center_global,
        UNIT_QUATERNION)

    com = np.copy(self._gym_env.robot.base_position)
    com[2] = 0.0
    self._gym_env.pybullet_client.resetBasePositionAndOrientation(
        self._projected_com_visual_body, com, UNIT_QUATERNION)

  @property
  def observation(self):
    return self._observation

  @property
  def env_step_counter(self):
    return self._step_counter
