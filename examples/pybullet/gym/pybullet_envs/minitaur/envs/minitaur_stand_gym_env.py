"""This file implements the gym environment of minitaur.

"""
import math
from operator import add
import time

from gym import spaces
import numpy as np
from pybullet_envs.minitaur.envs import minitaur_gym_env
import pybullet_data

ACTION_EPS = 0.01
# RANGE_OF_LEG_MOTION defines how far legs can rotate in both directions
# (1.0 means rotation pi/2 (radians) in both directions).
RANGE_OF_LEG_MOTION = 0.7
# LIMIT_FALLEN defines when to consider robot fallen to the ground.
# This is the vertical component of the robot's front vector (unit vector).
# 0.0 represents body of the robot being horizontal, 1.0 means vertical.
LIMIT_FALLEN = 0.7


class MinitaurStandGymEnv(minitaur_gym_env.MinitaurGymEnv):
  """The gym environment for the minitaur and a ball.

  It simulates the standing up behavior of a minitaur, a quadruped robot. The
  state space include the angles, velocities and torques for all the motors and
  the action space is the desired motor angle for each motor. The reward
  function is based on how long the minitaur stays standing up.

  """
  metadata = {"render.modes": ["human", "rgb_array"], "video.frames_per_second": 50}

  def __init__(self,
               urdf_root=pybullet_data.getDataPath(),
               action_repeat=1,
               observation_noise_stdev=minitaur_gym_env.SENSOR_NOISE_STDDEV,
               self_collision_enabled=True,
               motor_velocity_limit=np.inf,
               pd_control_enabled=False,
               render=False):
    """Initialize the minitaur standing up gym environment.

    Args:
      urdf_root: The path to the urdf data folder.
      action_repeat: The number of simulation steps before actions are applied.
      observation_noise_stdev: The standard deviation of observation noise.
      self_collision_enabled: Whether to enable self collision in the sim.
      motor_velocity_limit: The velocity limit of each motor.
      pd_control_enabled: Whether to use PD controller for each motor.
      render: Whether to render the simulation.
    """
    super(MinitaurStandGymEnv, self).__init__(urdf_root=urdf_root,
                                              action_repeat=action_repeat,
                                              observation_noise_stdev=observation_noise_stdev,
                                              self_collision_enabled=self_collision_enabled,
                                              motor_velocity_limit=motor_velocity_limit,
                                              pd_control_enabled=pd_control_enabled,
                                              accurate_motor_model_enabled=True,
                                              motor_overheat_protection=True,
                                              render=render)
    # Set the action dimension to 1, and reset the action space.
    action_dim = 1
    action_high = np.array([self._action_bound] * action_dim)
    self.action_space = spaces.Box(-action_high, action_high)

  def _stand_up(self):
    """Make the robot stand up to its two legs when started on 4 legs.

    This method is similar to the step function, but instead of using the action
    provided, it uses a hand-coded policy to make the robot stand up to its
    two legs. Once the robot is vertical enough it exits and leaves the
    environment to the typical step function that uses agent's actions.

    Returns:
      observations: The angles, velocities and torques of all motors.
      reward: The reward for the current state-action pair.
      done: Whether the episode has ended.
      info: A dictionary that stores diagnostic information.
    """
    for t in range(5000):
      if self._is_render:
        base_pos = self.minitaur.GetBasePosition()
        self._pybullet_client.resetDebugVisualizerCamera(self._cam_dist, self._cam_yaw,
                                                         self._cam_pitch, base_pos)
      state = self._get_true_observation()
      action = self._policy_flip(t, state[24:28])
      self.minitaur.ApplyAction(action)
      self._pybullet_client.stepSimulation()
      self.minitaur.ReceiveObservation()
      if self._is_render:
        time.sleep(self._time_step)
      self._env_step_counter += 1
      reward = self._reward()
      if reward > 0.9:
        break
    done = self._termination()
    return np.array(self._get_observation()), reward, done, {}

  def step(self, action):
    # At start, use policy_flip to lift the robot to its two legs. After the
    # robot reaches the lift up stage, give control back to the agent by
    # returning the current state and reward.
    if self._env_step_counter < 1:
      return self._stand_up()
    return super(MinitaurStandGymEnv, self).step(action)

  def _reward(self):
    """Reward function for standing up pose.

    Returns:
      reward: A number between -1 and 1 according to how vertical is the body of
        the robot.
    """
    orientation = self.minitaur.GetBaseOrientation()
    rot_matrix = self._pybullet_client.getMatrixFromQuaternion(orientation)
    local_front_vec = rot_matrix[6:9]
    alignment = abs(np.dot(np.asarray([1, 0, 0]), np.asarray(local_front_vec)))
    return alignment**4

  def _termination(self):
    if self._is_horizontal():
      return True
    return False

  def _is_horizontal(self):
    """Decide whether minitaur is almost parallel to the ground.

    This method is used in experiments where the robot is learning to stand up.

    Returns:
      Boolean value that indicates whether the minitaur is almost parallel to
      the ground.
    """
    orientation = self.minitaur.GetBaseOrientation()
    rot_matrix = self._pybullet_client.getMatrixFromQuaternion(orientation)
    front_z_component = rot_matrix[6]
    return abs(front_z_component) < LIMIT_FALLEN

  def _transform_action_to_motor_command(self, action):
    """Method to transform the one dimensional action to rotate bottom two legs.

    Args:
      action: A double between -1 and 1, where 0 means keep the legs parallel
        to the body.
    Returns:
      actions: The angles for all motors.
    Raises:
      ValueError: The action dimension is not the same as the number of motors.
      ValueError: The magnitude of actions is out of bounds.
    """
    action = action[0]
    # Scale the action from [-1 to 1] to [-range to +range] (angle in radians).
    action *= RANGE_OF_LEG_MOTION
    action_all_legs = [
        math.pi,  # Upper leg pointing up.
        0,
        0,  # Bottom leg pointing down.
        math.pi,
        0,  # Upper leg pointing up.
        math.pi,
        math.pi,  # Bottom leg pointing down.
        0
    ]
    action_all_legs = [angle - 0.7 for angle in action_all_legs]

    # Use the one dimensional action to rotate both bottom legs.
    action_delta = [0, 0, -action, action, 0, 0, action, -action]
    action_all_legs = map(add, action_all_legs, action_delta)
    return list(action_all_legs)

  def _policy_flip(self, time_step, orientation):
    """Hand coded policy to make the minitaur stand up to its two legs.

    This method is the hand coded policy that uses sine waves and orientation
    of the robot to make it stand up to its two legs. It is composed of these
    behaviors:
    - Rotate bottom legs to always point to the ground
    - Rotate upper legs the other direction so that they point to the sky when
    the robot is standing up, and they point to the ground when the robot is
    horizontal.
    - Shorten the bottom 2 legs
    - Shorten the other two legs, then when the sine wave hits its maximum,
    extend the legs pushing the robot up.

    Args:
      time_step: The time (or frame number) used for sine function.
      orientation: Quaternion specifying the orientation of the body.

    Returns:
      actions: The angles for all motors.
    """
    # Set the default behavior (stand on 4 short legs).
    shorten = -0.7
    a0 = math.pi / 2 + shorten
    a1 = math.pi / 2 + shorten
    a2 = math.pi / 2 + shorten
    a3 = math.pi / 2 + shorten
    a4 = math.pi / 2 + shorten
    a5 = math.pi / 2 + shorten
    a6 = math.pi / 2 + shorten
    a7 = math.pi / 2 + shorten

    # Rotate back legs to point to the ground independent of the orientation.
    rot_matrix = self._pybullet_client.getMatrixFromQuaternion(orientation)
    local_up = rot_matrix[6:]
    multiplier = np.dot(np.asarray([0, 0, 1]), np.asarray(local_up))
    a2 -= (1 - multiplier) * (math.pi / 2)
    a3 += (1 - multiplier) * (math.pi / 2)
    a6 += (1 - multiplier) * (math.pi / 2)
    a7 -= (1 - multiplier) * (math.pi / 2)

    # Rotate front legs the other direction, so that it points up when standing.
    a0 += (1 - multiplier) * (math.pi / 2)
    a1 -= (1 - multiplier) * (math.pi / 2)
    a4 -= (1 - multiplier) * (math.pi / 2)
    a5 += (1 - multiplier) * (math.pi / 2)

    # Periodically push the upper legs to stand up.
    speed = 0.01
    intensity = 1.9
    # Lower the signal a little, so that it becomes positive only for a short
    # amount time.
    lower_signal = -0.94
    signal_unit = math.copysign(intensity, math.sin(time_step * speed) + lower_signal)
    # Only extend the leg, don't shorten.
    if signal_unit < 0:
      signal_unit = 0

    # Only apply it after some time.
    if time_step * speed > math.pi:
      a0 += signal_unit
      a1 += signal_unit
      a4 += signal_unit
      a5 += signal_unit
    joint_values = [a0, a1, a2, a3, a4, a5, a6, a7]
    return joint_values
