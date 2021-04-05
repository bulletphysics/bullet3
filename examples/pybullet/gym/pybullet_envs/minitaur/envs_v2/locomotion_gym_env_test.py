# Lint as: python3
r"""Tests for locomotion_gym_env.


"""

import math
import random

import gin
import mock
import numpy as np
import tensorflow.compat.v1 as tf
from absl.testing import parameterized

from pybullet_envs.minitaur.envs_v2 import locomotion_gym_env
from pybullet_envs.minitaur.envs_v2.evaluation import metric as metric_lib
from pybullet_envs.minitaur.envs_v2.scenes import scene_base
from pybullet_envs.minitaur.envs_v2.scenes import simple_scene
from pybullet_envs.minitaur.envs_v2.tasks import task_interface
from pybullet_envs.minitaur.envs_v2.utilities import env_utils
from pybullet_envs.minitaur.robots import autonomous_object
from pybullet_envs.minitaur.robots import minitaur_v2
import pybullet_data as pd

import unittest


_POSITION_GAIN = 1.0
_VELOCITY_GAIN = 0.015
_CONTROL_LATENCY = 0.015
_CONFIG_FILE = (pd.getDataPath()+'/configs/minitaur_gym_env.gin')
_CONFIG_FILE_NEW_ROBOT = (pd.getDataPath()+'/configs_v2/base/laikago_reactive.gin')


class TestTask(task_interface.Task):
  """A step counter task for test purpose."""

  def __init__(self):
    self._counter = 0

  def reset(self, env):
    del env
    self._counter = 0

  def reward(self, env):
    del env  # TODO(b/154635313): resolve the API mismatch
    self._counter += 1
    return self._counter

  def update(self, env):
    del env  # TODO(b/154635313): resolve the API mismatch

  def done(self, env):
    del env  # TODO(b/154635313): resolve the API mismatch
    return False


class LocomotionGymEnvTest(tf.test.TestCase, parameterized.TestCase):

  def setUp(self):
    super().setUp()
    gin.clear_config()

  def test_env_from_gin(self):
    # TODO(sehoonha) rename locomotion_gym_*test.gin to locomotion_gym_*.gin
    gin.parse_config_file(_CONFIG_FILE)
    env = locomotion_gym_env.LocomotionGymEnv()
    self.assertIsInstance(env.robot, minitaur_v2.Minitaur)
    # The robot will stand on the ground.
    self.assertNear(env.robot.base_position[2], 0.25, 5e-2)

  def test_reset_gym(self):
    gin.parse_config_file(_CONFIG_FILE)
    env = locomotion_gym_env.LocomotionGymEnv(task=None)

    desired_init_motor_angle = math.pi / 2
    action_dim = len(env.action_space.high)
    observations = env.reset(initial_motor_angles=[desired_init_motor_angle] *
                             action_dim)
    observations = env_utils.flatten_observations(observations)
    self.assertEqual(observations.size, 12)
    self.assertNear(observations[0], 0, 1e-2)
    self.assertNear(observations[4], desired_init_motor_angle, 2e-1)

  def test_step_gym(self):
    gin.parse_config_file(_CONFIG_FILE)
    env = locomotion_gym_env.LocomotionGymEnv(task=TestTask())

    desired_motor_angle = math.pi / 3
    steps = 1000
    action_dim = len(env.action_space.high)
    for _ in range(steps):
      observations, reward, done, _ = env.step([desired_motor_angle] *
                                               action_dim)
      observations = env_utils.flatten_observations(observations)

    self.assertFalse(done)
    self.assertEqual(reward, steps)
    self.assertEqual(observations.size, 12)
    self.assertNear(observations[0], 0, 1e-2)
    self.assertNear(observations[4], desired_motor_angle, 2e-1)
    np.testing.assert_allclose(env._last_action,
                               [desired_motor_angle] * action_dim, 2e-1)

  def test_scene(self):
    gin.parse_config_file(_CONFIG_FILE)
    data_root = 'urdf/'
    env = locomotion_gym_env.LocomotionGymEnv(
        task=TestTask(), scene=simple_scene.SimpleScene(data_root=data_root))
    desired_motor_angle = math.pi / 3
    steps = 2
    action_dim = len(env.action_space.high)
    for _ in range(steps):
      _, reward, _, _ = env.step([desired_motor_angle] * action_dim)
    self.assertEqual(reward, steps)

  def test_except_on_invalid_config(self):
    gin.parse_config_file(_CONFIG_FILE)
    gin.bind_parameter('SimulationParameters.num_action_repeat', 0)
    with self.assertRaises(ValueError):
      locomotion_gym_env.LocomotionGymEnv(task=None)

  def test_no_scene(self):
    gin.parse_config_file(_CONFIG_FILE)
    env = locomotion_gym_env.LocomotionGymEnv(task=None, scene=None)

    # The robot will free fall.
    self.assertNear(env.robot.base_position[2], 0.15, 5e-2)

  def test_seed_draw_with_np(self):
    gin.parse_config_file(_CONFIG_FILE)
    env = locomotion_gym_env.LocomotionGymEnv(task=None)
    # first draw
    env.seed(42)
    nums1 = []
    for _ in range(3):
      nums1.append(env.np_random.randint(2**31 - 1))
    # second draw
    env.seed(42)
    nums2 = []
    for _ in range(3):
      nums2.append(env.np_random.randint(2**31 - 1))
    self.assertListEqual(nums1, nums2)

  def test_get_observations(self):
    gin.parse_config_file(_CONFIG_FILE)
    env = locomotion_gym_env.LocomotionGymEnv(task=None)
    desired_init_motor_angle = math.pi / 2
    action_dim = len(env.action_space.high)
    observations = env.reset(initial_motor_angles=[desired_init_motor_angle] *
                             action_dim)
    self.assertLen(observations, 2)
    self.assertLen(observations['IMU'], 4)
    self.assertNear(observations['IMU'][0], 0, 1e-2)
    self.assertLen(observations['MotorAngle'], 8)
    self.assertNear(observations['MotorAngle'][0], desired_init_motor_angle,
                    2e-1)

  

  

  def test_step_with_dynamic_objects(self):
    gin.parse_config_file(_CONFIG_FILE_NEW_ROBOT)

    gin.parse_config([
        'AutonomousObject.urdf_file = "urdf/mug.urdf"',
        'SceneBase.dynamic_objects = [@AutonomousObject(), @AutonomousObject()]',
        'LocomotionGymEnv.scene = @SceneBase()',
    ])
    env = locomotion_gym_env.LocomotionGymEnv()

    self.assertLen(env.scene.dynamic_objects, 2)
    for obj in env.scene.dynamic_objects:
      self.assertIsInstance(obj, autonomous_object.AutonomousObject)

    # Replace dynamic objects with mocks for step tests.
    mock_objects = [
        mock.create_autospec(autonomous_object.AutonomousObject),
        mock.create_autospec(autonomous_object.AutonomousObject)
    ]
    env.scene._type_to_objects_dict[
        scene_base.ObjectType.DYNAMIC_OBJECT] = mock_objects
    env.step(env.robot.action_space.sample())

    expected_update_calls = [
        mock.call(i * env.sim_time_step, mock.ANY)
        for i in range(env.num_action_repeat)
    ]
    expected_apply_action_calls = [
        mock.call(autonomous_object.AUTONOMOUS_ACTION)
        for i in range(env.num_action_repeat)
    ]
    expected_receive_observation_calls = [
        mock.call() for i in range(env.num_action_repeat)
    ]

    for mock_obj in mock_objects:
      mock_obj.pre_control_step.assert_called_once_with(
          autonomous_object.AUTONOMOUS_ACTION)
      self.assertEqual(mock_obj.update.call_args_list, expected_update_calls)
      self.assertEqual(mock_obj.apply_action.call_args_list,
                       expected_apply_action_calls)
      self.assertEqual(mock_obj.receive_observation.call_args_list,
                       expected_receive_observation_calls)
      mock_obj.post_control_step.assert_called_once_with()

 
  def test_env_metrics(self):
    gin.parse_config_file(_CONFIG_FILE_NEW_ROBOT)
    env = locomotion_gym_env.LocomotionGymEnv()
    metric_logger = env.metric_logger
    test_metric_1 = metric_logger.create_scalar_metric(
        'test_metric_1',
        scope=metric_lib.MetricScope.DEBUG,
        single_ep_aggregator=np.mean)
    test_metric_1.report(22)

    test_metric_2 = metric_logger.create_scalar_metric(
        'test_metric_2',
        scope=metric_lib.MetricScope.PERFORMANCE,
        single_ep_aggregator=np.max)
    test_metric_2.report(15)
    test_metric_2.report(16)

    all_metrics = metric_logger.get_episode_metrics()

    self.assertDictEqual(all_metrics, {
        'DEBUG/test_metric_1': 22,
        'PERFORMANCE/test_metric_2': 16
    })

    env.reset()

    all_metrics = metric_logger.get_episode_metrics()
    self.assertDictEqual(all_metrics, {})


if __name__ == '__main__':
   tf.test.main()
