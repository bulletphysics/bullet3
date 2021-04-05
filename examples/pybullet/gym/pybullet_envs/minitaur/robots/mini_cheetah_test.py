"""Tests for pybullet_envs.minitaur.robots.mini_cheetah.

blaze test -c opt
//robotics/reinforcement_learning/minitaur/robots:mini_cheetah_test
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import math
import numpy as np
from pybullet_envs.minitaur.envs import bullet_client
from pybullet_envs.minitaur.robots import mini_cheetah
from google3.testing.pybase import googletest

PI = math.pi
NUM_STEPS = 500
TIME_STEP = 0.002
INIT_MOTOR_ANGLES = [0, -0.6, 1.4] * 4


class MiniCheetahTest(googletest.TestCase):

  def test_init(self):
    pybullet_client = bullet_client.BulletClient()
    pybullet_client.enable_cns()
    robot = mini_cheetah.MiniCheetah(
        pybullet_client=pybullet_client, time_step=TIME_STEP, on_rack=True)
    self.assertIsNotNone(robot)

  def test_static_pose_on_rack(self):
    pybullet_client = bullet_client.BulletClient()
    pybullet_client.enable_cns()
    pybullet_client.resetSimulation()
    pybullet_client.setPhysicsEngineParameter(numSolverIterations=60)
    pybullet_client.setTimeStep(TIME_STEP)
    pybullet_client.setGravity(0, 0, -10)

    robot = (
        mini_cheetah.MiniCheetah(
            pybullet_client=pybullet_client,
            action_repeat=5,
            time_step=0.002,
            on_rack=True))
    robot.Reset(
        reload_urdf=False,
        default_motor_angles=INIT_MOTOR_ANGLES,
        reset_time=0.5)
    for _ in range(NUM_STEPS):
      robot.Step(INIT_MOTOR_ANGLES)
      motor_angles = robot.GetMotorAngles()
      np.testing.assert_array_almost_equal(
          motor_angles, INIT_MOTOR_ANGLES, decimal=2)


if __name__ == '__main__':
  googletest.main()
