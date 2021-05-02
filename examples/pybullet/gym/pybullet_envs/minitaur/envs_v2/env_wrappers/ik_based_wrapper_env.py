"""A wrapped Quadruped with Inverse Kinematics based controller."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import gin
import gym
import numpy as np
from pybullet_envs.minitaur.robots import vision60
from pybullet_envs.minitaur.robots.utilities import kinematics

ACTION_DIM_PER_LEG = 3
ACTION_DIM_BASE = 7
ACTION_DIM_TOTAL = vision60.NUM_LEGS * ACTION_DIM_PER_LEG + ACTION_DIM_BASE


@gin.configurable
class IKBasedWrapperEnv(object):
  """An env using IK to convert toe positions to joint angles."""

  def __init__(self,
               gym_env,
               toe_indices=(3, 7, 11, 15),
               abduction_motor_ids=(0, 3, 6, 9)):
    """Initialzes the wrapped env.

    Args:
      gym_env: An instance of LocomotionGymEnv.
      toe_indices: A list of four pybullet joint indices for the four toes. [3,
        7, 11, 15] for the vision60.
      abduction_motor_ids: A list of four pybullet joint indices for the four
        abuduction motors. [0, 3, 6, 9] for the vision60.
    """
    lower_bound = np.array([-1.0] * ACTION_DIM_TOTAL)
    upper_bound = np.array([1.0] * ACTION_DIM_TOTAL)
    self._gym_env = gym_env
    self.action_space = gym.spaces.Box(lower_bound, upper_bound)
    self._toe_ids = toe_indices
    self._abduction_motor_ids = abduction_motor_ids

  def __getattr__(self, attr):
    return getattr(self._gym_env, attr)

  def get_toe_indices(self):
    return self._toe_ids

  def _joint_angles_from_toe_positions_and_base_pose(self, ik_actions):
    """Uses Inverse Kinematics to calculate jont angles.

    Args:
      ik_actions: The action should be local (x, y, z) for each toe. action for
        each leg [x, y, z] in a local frame. This local frame is transformed
        relative to the COM frame using a given translation, and rotation. The
        total action space would be 3 + 4 + 3 * ACTION_DIM_PER_LEG = 16.

    Returns:
      A list of joint angles.
    """
    assert len(ik_actions) == ACTION_DIM_TOTAL

    base_translation_index = vision60.NUM_LEGS * ACTION_DIM_PER_LEG
    base_rotation_index = vision60.NUM_LEGS * ACTION_DIM_PER_LEG + 3

    base_translation = ik_actions[
        base_translation_index:base_translation_index + 3]
    base_rotation = ik_actions[base_rotation_index:base_rotation_index + 4]
    desired_joint_angles = []
    for i in range(vision60.NUM_LEGS):
      local_toe_pos = ik_actions[i * ACTION_DIM_PER_LEG:i * ACTION_DIM_PER_LEG +
                                 ACTION_DIM_PER_LEG]
      leg_joint_ids = [
          self._abduction_motor_ids[i], self._abduction_motor_ids[i] + 1,
          self._abduction_motor_ids[i] + 2
      ]

      desired_joint_angles.extend(
          kinematics.joint_angles_from_link_position(
              robot=self._gym_env.robot,
              link_position=local_toe_pos,
              link_id=self._toe_ids[i],
              joint_ids=leg_joint_ids,
              base_translation=base_translation,
              base_rotation=base_rotation))

    return desired_joint_angles

  def step(self, action):
    """Steps the wrapped environment.

    Args:
      action: Numpy array. The input action from an NN agent.

    Returns:
      The tuple containing the modified observation, the reward, the epsiode end
      indicator.

    Raises:
      ValueError if input action is None.

    """
    if action is None:
      raise ValueError('Action cannot be None')

    desired_joint_angles = self._joint_angles_from_toe_positions_and_base_pose(
        ik_actions=action)
    observation, reward, done, _ = self._gym_env.step(desired_joint_angles)

    return observation, reward, done, _
