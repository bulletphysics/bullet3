"""Tests for pybullet_envs.minitaur.envs.locomotion_gym_config."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import gin
from pybullet_envs.minitaur.envs_v2 import locomotion_gym_config
import tensorflow.compat.v1 as tf
from absl.testing import parameterized



class LocomotionGymConfigTest(tf.test.TestCase, parameterized.TestCase):

  def testSimulationParametersFromGinString(self):
    config_text = (
        'import pybullet_envs.minitaur'
        '.envs_v2.locomotion_gym_config\n'
        'locomotion_gym_config.SimulationParameters.sim_time_step_s = 0.005\n'
        'locomotion_gym_config.SimulationParameters.camera_distance = 5.0\n'
        'locomotion_gym_config.SimulationParameters.camera_yaw = 10\n'
        'locomotion_gym_config.SimulationParameters.camera_pitch = -50\n'
    )
    gin.parse_config(config_text)

    cfg = locomotion_gym_config.SimulationParameters()
    self.assertEqual(cfg.sim_time_step_s, 0.005)
    self.assertFalse(cfg.enable_hard_reset)
    self.assertEqual(cfg.camera_distance, 5.0)
    self.assertEqual(cfg.camera_yaw, 10)
    self.assertEqual(cfg.camera_pitch, -50)

  def testScalarFieldFromGinString(self):
    config_text = (
        'import pybullet_envs.minitaur'
        '.envs_v2.locomotion_gym_config\n'
        'locomotion_gym_config.ScalarField.name = "MotorUpperLimit"\n'
        'locomotion_gym_config.ScalarField.upper_bound = 1.0\n'
        'locomotion_gym_config.ScalarField.lower_bound = -1.0\n'
    )
    gin.parse_config(config_text)

    cfg = locomotion_gym_config.ScalarField()
    self.assertEqual(cfg.name, 'MotorUpperLimit')
    self.assertEqual(cfg.upper_bound, 1.0)
    self.assertEqual(cfg.lower_bound, -1.0)

  def testLocomotionGymConfigFromGinString(self):
    config_text = (
        'import pybullet_envs.minitaur'
        '.envs_v2.locomotion_gym_config\n'
        # SimulationParameters
        'locomotion_gym_config.SimulationParameters.sim_time_step_s = 0.005\n'
        # Actions
        'Motor/locomotion_gym_config.ScalarField.name = "MotorUpperLimit"\n'
        'Motor/locomotion_gym_config.ScalarField.upper_bound = 1.0\n'
        'Motor/locomotion_gym_config.ScalarField.lower_bound = -1.0\n'
        # LocomotionGymConfigs
        'locomotion_gym_config.LocomotionGymConfig.simulation_parameters = '
        '@locomotion_gym_config.SimulationParameters()\n'
        'locomotion_gym_config.LocomotionGymConfig.actions = ['
        '@Motor/locomotion_gym_config.ScalarField()]\n'
        'locomotion_gym_config.LocomotionGymConfig.ignored_sensor_list = ['
        '"Collisions"]\n')
    gin.parse_config(config_text)

    cfg = locomotion_gym_config.LocomotionGymConfig()
    self.assertEqual(cfg.simulation_parameters.sim_time_step_s, 0.005)
    self.assertEqual(cfg.actions[0].upper_bound, 1.0)
    self.assertEqual(cfg.ignored_sensor_list, ['Collisions'])


if __name__ == '__main__':
  tf.test.main()
