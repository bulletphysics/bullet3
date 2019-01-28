"""This file implements the gym environment of minitaur.

"""
import math
import random

import os,  inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)

from gym import spaces
import numpy as np
from pybullet_envs.minitaur.envs import minitaur_gym_env
import pybullet_data

GOAL_DISTANCE_THRESHOLD = 0.8
GOAL_REWARD = 1000.0
REWARD_SCALING = 1e-3
INIT_BALL_ANGLE = math.pi / 3
INIT_BALL_DISTANCE = 5.0
ACTION_EPS = 0.01


class MinitaurBallGymEnv(minitaur_gym_env.MinitaurGymEnv):
  """The gym environment for the minitaur and a ball.

  It simulates a minitaur (a quadruped robot) and a ball. The state space
  includes the angle and distance of the ball relative to minitaur's base.
  The action space is a steering command. The reward function is based
  on how far the ball is relative to the minitaur's base.

  """

  def __init__(self,
               urdf_root=pybullet_data.getDataPath(),
               self_collision_enabled=True,
               pd_control_enabled=False,
               leg_model_enabled=True,
               on_rack=False,
               render=False):
    """Initialize the minitaur and ball gym environment.

    Args:
      urdf_root: The path to the urdf data folder.
      self_collision_enabled: Whether to enable self collision in the sim.
      pd_control_enabled: Whether to use PD controller for each motor.
      leg_model_enabled: Whether to use a leg motor to reparameterize the action
        space.
      on_rack: Whether to place the minitaur on rack. This is only used to debug
        the walking gait. In this mode, the minitaur's base is hanged midair so
        that its walking gait is clearer to visualize.
      render: Whether to render the simulation.
    """
    super(MinitaurBallGymEnv, self).__init__(
        urdf_root=urdf_root,
        self_collision_enabled=self_collision_enabled,
        pd_control_enabled=pd_control_enabled,
        leg_model_enabled=leg_model_enabled,
        on_rack=on_rack,
        render=render)
    self._cam_dist = 2.0
    self._cam_yaw = -70
    self._cam_pitch = -30
    self.action_space = spaces.Box(np.array([-1]), np.array([1]))
    self.observation_space = spaces.Box(np.array([-math.pi, 0]),
                                        np.array([math.pi, 100]))

  def reset(self):
    self._ball_id = 0
    super(MinitaurBallGymEnv, self).reset()
    self._init_ball_theta = random.uniform(-INIT_BALL_ANGLE, INIT_BALL_ANGLE)
    self._init_ball_distance = INIT_BALL_DISTANCE
    self._ball_pos = [self._init_ball_distance *
                      math.cos(self._init_ball_theta),
                      self._init_ball_distance *
                      math.sin(self._init_ball_theta), 1]
    self._ball_id = self._pybullet_client.loadURDF(
        "%s/sphere_with_restitution.urdf" % self._urdf_root, self._ball_pos)
    return self._get_observation()

  def _get_observation(self):
    world_translation_minitaur, world_rotation_minitaur = (
        self._pybullet_client.getBasePositionAndOrientation(
            self.minitaur.quadruped))
    world_translation_ball, world_rotation_ball = (
        self._pybullet_client.getBasePositionAndOrientation(self._ball_id))
    minitaur_translation_world, minitaur_rotation_world = (
        self._pybullet_client.invertTransform(world_translation_minitaur,
                                              world_rotation_minitaur))
    minitaur_translation_ball, _ = (
        self._pybullet_client.multiplyTransforms(minitaur_translation_world,
                                                 minitaur_rotation_world,
                                                 world_translation_ball,
                                                 world_rotation_ball))
    distance = math.sqrt(minitaur_translation_ball[0]**2 +
                         minitaur_translation_ball[1]**2)
    angle = math.atan2(minitaur_translation_ball[0],
                       minitaur_translation_ball[1])
    self._observation = [angle - math.pi / 2, distance]
    return self._observation

  def _transform_action_to_motor_command(self, action):
    if self._leg_model_enabled:
      for i, action_component in enumerate(action):
        if not (-self._action_bound - ACTION_EPS <=
                action_component <= self._action_bound + ACTION_EPS):
          raise ValueError("{}th action {} out of bounds.".format
                           (i, action_component))
      action = self._apply_steering_to_locomotion(action)
      action = self.minitaur.ConvertFromLegModel(action)
    return action

  def _apply_steering_to_locomotion(self, action):
    # A hardcoded feedforward walking controller based on sine functions.
    amplitude_swing = 0.5
    amplitude_extension = 0.5
    speed = 200
    steering_amplitude = 0.5 * action[0]
    t = self.minitaur.GetTimeSinceReset()
    a1 = math.sin(t * speed) * (amplitude_swing + steering_amplitude)
    a2 = math.sin(t * speed + math.pi) * (amplitude_swing - steering_amplitude)
    a3 = math.sin(t * speed) * amplitude_extension
    a4 = math.sin(t * speed + math.pi) * amplitude_extension
    action = [a1, a2, a2, a1, a3, a4, a4, a3]
    return action

  def _distance_to_ball(self):
    world_translation_minitaur, _ = (
        self._pybullet_client.getBasePositionAndOrientation(
            self.minitaur.quadruped))
    world_translation_ball, _ = (
        self._pybullet_client.getBasePositionAndOrientation(
            self._ball_id))
    distance = math.sqrt(
        (world_translation_ball[0] - world_translation_minitaur[0])**2 +
        (world_translation_ball[1] - world_translation_minitaur[1])**2)
    return distance

  def _goal_state(self):
    return self._observation[1] < GOAL_DISTANCE_THRESHOLD

  def _reward(self):
    reward = -self._observation[1]
    if self._goal_state():
      reward += GOAL_REWARD
    return reward * REWARD_SCALING

  def _termination(self):
    if self._goal_state():
      return True
    return False
