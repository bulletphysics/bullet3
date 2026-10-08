# Bullet Continuous Collision Detection and Physics Library
# http://bulletphysics.org
#
# This software is provided 'as-is', without any express or implied warranty.
# In no event will the authors be held liable for any damages arising from the use of this software.
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it freely,
# subject to the following restrictions:
#
# 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
# 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
# 3. This notice may not be removed or altered from any source distribution.
"""Native regressions for DeepMimic's relative angular velocity helpers."""

import importlib.util
import math
from pathlib import Path
import unittest

import numpy as np
import pybullet


def load_interpolator(filename, class_name):
  source = Path(__file__).resolve().parents[1] / "gym/pybullet_envs/deep_mimic/env" / filename
  spec = importlib.util.spec_from_file_location(source.stem, source)
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return getattr(module, class_name)


INTERPOLATORS = [
    load_interpolator("humanoid_pose_interpolator.py", "HumanoidPoseInterpolator"),
    load_interpolator("quadrupedPoseInterpolator.py", "QuadrupedPoseInterpolator"),
]


class TestPoseInterpolatorAngularVelocity(unittest.TestCase):

  def test_relative_velocity_follows_shortest_arc(self):
    duration = 0.1
    axis = np.array([1., 2., 3.]) / math.sqrt(14.)
    for interpolator_class in INTERPOLATORS:
      interpolator = interpolator_class()
      for start_euler in ([0., 0., 0.], [0.4, -0.3, -2.]):
        start = pybullet.getQuaternionFromEuler(start_euler)
        for angle in (0., 0.2, -0.2, 2.5, -2.5, math.pi - 0.01, math.pi + 0.01, -math.pi + 0.01,
                      -math.pi - 0.01, 4., -4.):
          delta = pybullet.getQuaternionFromAxisAngle(axis, angle)
          end = pybullet.multiplyTransforms([0, 0, 0], start, [0, 0, 0], delta)[1]
          expected = axis * math.atan2(math.sin(angle), math.cos(angle)) / duration
          for start_sign, end_sign in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
            with self.subTest(interpolator=interpolator_class.__name__,
                              start=start_euler,
                              angle=angle,
                              signs=(start_sign, end_sign)):
              velocity = interpolator.ComputeAngVelRel([start_sign * q for q in start],
                                                       [end_sign * q for q in end], duration,
                                                       pybullet)
              np.testing.assert_allclose(velocity, expected, atol=2e-4, rtol=2e-5)

  def test_relative_velocity_matches_native_slerp_derivative(self):
    pairs = [
        (pybullet.getQuaternionFromEuler([0.4, -0.3,
                                          -2.]), pybullet.getQuaternionFromEuler([0.4, -0.3,
                                                                                  2.]), 0.1),
        ([-0.5785699654872136, -0.3038449118109436, -0.114634350382299, 0.7481938454138425],
         [0.6677329540252686, 0.33715036511421204, -0.24929332733154297, 0.6150732636451721], 0.2),
    ]
    half_step = 0.002
    for start, end, duration in pairs:

      def rotation(fraction):
        quat = pybullet.getQuaternionSlerp(start, end, fraction)
        return np.array(pybullet.getMatrixFromQuaternion(quat)).reshape(3, 3)

      derivative = (rotation(0.5 + half_step) - rotation(0.5 - half_step)) / (2 * half_step *
                                                                              duration)
      angular_matrix = derivative @ rotation(0.5).T
      world_velocity = np.array([angular_matrix[2, 1], angular_matrix[0, 2], angular_matrix[1, 0]])
      start_rotation = np.array(pybullet.getMatrixFromQuaternion(start)).reshape(3, 3)
      expected = start_rotation.T @ world_velocity
      for interpolator_class in INTERPOLATORS:
        with self.subTest(interpolator=interpolator_class.__name__, start=start):
          velocity = interpolator_class().ComputeAngVelRel(start, end, duration, pybullet)
          np.testing.assert_allclose(velocity, expected, atol=2e-3, rtol=1e-4)

  def test_duration_range_and_scalar_dtype_are_preserved(self):
    start = np.asarray(pybullet.getQuaternionFromEuler([0.4, -0.3, -2.]))
    axis = np.array([1., 2., 3.]) / math.sqrt(14.)
    delta = pybullet.getQuaternionFromAxisAngle(axis, 0.6)
    end = np.asarray(pybullet.multiplyTransforms([0, 0, 0], start, [0, 0, 0], delta)[1])
    durations = (0.2, np.float32(0.2), np.float64(0.2), 1e-20, 1e-40, 1e-100, 1e100, -0.2)
    for interpolator_class in INTERPOLATORS:
      for dtype in (np.float32, np.float64):
        for duration in durations:
          with self.subTest(interpolator=interpolator_class.__name__,
                            dtype=dtype,
                            duration=duration,
                            scalar_type=type(duration)):
            velocity = interpolator_class().ComputeAngVelRel(start.astype(dtype),
                                                             end.astype(dtype), duration, pybullet)
            self.assertIsInstance(velocity, list)
            array = np.asarray(velocity)
            expected_dtype = np.asarray([1.0 / duration]).dtype
            self.assertEqual(array.dtype, expected_dtype)
            self.assertTrue(np.isfinite(array).all())
            self.assertTrue((array != 0).any())
            expected = axis * 0.6 / float(duration)
            np.testing.assert_allclose(array, expected, rtol=1e-4, atol=0.)

  def test_independent_orientation_scales_keep_body_coordinates(self):
    start = np.asarray(pybullet.getQuaternionFromEuler([0.4, -0.3, -2.]))
    axis = np.array([1., 2., 3.]) / math.sqrt(14.)
    delta = pybullet.getQuaternionFromAxisAngle(axis, 0.6)
    end = np.asarray(pybullet.multiplyTransforms([0, 0, 0], start, [0, 0, 0], delta)[1])
    expected = axis * 0.6 / 0.2
    for interpolator_class in INTERPOLATORS:
      for dtype in (np.float32, np.float64):
        for start_scale in (0.5, 1., 2.):
          for end_scale in (0.5, 1., 2.):
            for start_sign, end_sign in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
              with self.subTest(interpolator=interpolator_class.__name__,
                                dtype=dtype,
                                scales=(start_scale, end_scale),
                                signs=(start_sign, end_sign)):
                velocity = interpolator_class().ComputeAngVelRel(
                    (start_sign * start_scale * start).astype(dtype),
                    (end_sign * end_scale * end).astype(dtype), 0.2, pybullet)
                np.testing.assert_allclose(velocity, expected, rtol=1e-4, atol=0.)

  def test_zero_duration_remains_an_error(self):
    for interpolator_class in INTERPOLATORS:
      for end in ([0., 0., 0., 1.], pybullet.getQuaternionFromEuler([0., 0., 0.2])):
        with self.subTest(interpolator=interpolator_class.__name__, end=end):
          with self.assertRaises(ZeroDivisionError):
            interpolator_class().ComputeAngVelRel([0., 0., 0., 1.], end, 0., pybullet)

  def test_humanoid_slerp_joint_velocity_uses_same_arc(self):
    start = [0.1] + [0.] * 43
    for offset in (4, 8, 12, 16, 21, 25, 30, 35, 39):
      start[offset] = 1.
    end = start.copy()
    chest_start = pybullet.getQuaternionFromEuler([0, 0, -2.])
    chest_end = pybullet.getQuaternionFromEuler([0, 0, 2.])
    start[8:12] = [chest_start[3], *chest_start[:3]]
    end[8:12] = [chest_end[3], *chest_end[:3]]
    interpolator = INTERPOLATORS[0]()
    interpolator.Slerp(0.5, start, end, pybullet)
    expected = [0., 0., (4. - 2 * math.pi) / start[0]]
    np.testing.assert_allclose(interpolator._chestVel, expected, atol=2e-4)
    actual_rotation = np.array(pybullet.getMatrixFromQuaternion(interpolator._chestRot))
    expected_rotation = np.array(
        pybullet.getMatrixFromQuaternion(pybullet.getQuaternionSlerp(chest_start, chest_end, 0.5)))
    np.testing.assert_allclose(actual_rotation, expected_rotation)

  def test_relative_velocity_initializes_native_spherical_joint(self):
    client = pybullet.connect(pybullet.DIRECT)
    try:
      body = pybullet.createMultiBody(baseMass=0,
                                      baseCollisionShapeIndex=-1,
                                      baseVisualShapeIndex=-1,
                                      linkMasses=[1],
                                      linkCollisionShapeIndices=[-1],
                                      linkVisualShapeIndices=[-1],
                                      linkPositions=[[0, 0, 0]],
                                      linkOrientations=[[0, 0, 0, 1]],
                                      linkInertialFramePositions=[[0, 0, 0]],
                                      linkInertialFrameOrientations=[[0, 0, 0, 1]],
                                      linkParentIndices=[0],
                                      linkJointTypes=[pybullet.JOINT_SPHERICAL],
                                      linkJointAxis=[[0, 0, 1]],
                                      physicsClientId=client)
      pybullet.changeDynamics(body,
                              0,
                              localInertiaDiagonal=[1, 1, 1],
                              jointDamping=0,
                              physicsClientId=client)
      pybullet.setJointMotorControlMultiDof(body,
                                            0,
                                            pybullet.POSITION_CONTROL,
                                            targetPosition=[0, 0, 0, 1],
                                            force=[0, 0, 0],
                                            physicsClientId=client)
      pairs = [
          (pybullet.getQuaternionFromEuler([0.4, -0.3,
                                            -2.]), pybullet.getQuaternionFromEuler([0.4, -0.3,
                                                                                    2.]), 0.1),
          ([-0.5785699654872136, -0.3038449118109436, -0.114634350382299, 0.7481938454138425],
           [0.6677329540252686, 0.33715036511421204, -0.24929332733154297,
            0.6150732636451721], 0.2),
      ]
      time_step = 0.0001
      pybullet.setTimeStep(time_step, physicsClientId=client)
      for start, end, duration in pairs:
        expected = pybullet.getQuaternionSlerp(start, end, time_step / duration)
        expected_rotation = np.array(pybullet.getMatrixFromQuaternion(expected))
        for interpolator_class in INTERPOLATORS:
          with self.subTest(interpolator=interpolator_class.__name__, start=start):
            velocity = interpolator_class().ComputeAngVelRel(start, end, duration, pybullet)
            pybullet.resetJointStateMultiDof(body, 0, start, velocity, physicsClientId=client)
            pybullet.stepSimulation(physicsClientId=client)
            orientation = pybullet.getJointStateMultiDof(body, 0, physicsClientId=client)[0]
            actual_rotation = np.array(pybullet.getMatrixFromQuaternion(orientation))
            np.testing.assert_allclose(actual_rotation, expected_rotation, atol=1e-6)
    finally:
      pybullet.disconnect(client)


if __name__ == "__main__":
  unittest.main()
