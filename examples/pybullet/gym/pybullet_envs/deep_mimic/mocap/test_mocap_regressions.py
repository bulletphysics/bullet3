import unittest

import numpy as np

import camera
import quaternion
import transformation
from skeleton import Skeleton


class MocapRegressionTest(unittest.TestCase):
  def test_batched_inverse(self):
    q = np.arange(24.).reshape(2, 3, 4)
    np.testing.assert_array_equal(quaternion.qinverse(q), quaternion.qinverse(q.copy(), inplace=True))

  def test_per_frame_camera_poses(self):
    points = np.arange(24.).reshape(2, 4, 3)
    rotations = np.array([[[1., 0, 0, 0]], [[0., 0, 0, 1]]])
    offsets = np.array([[[1., 2, 3]], [[4., 5, 6]]])
    expected = np.stack([camera.camera_to_world(x, r[0], t) for x, r, t in zip(points, rotations, offsets)])
    np.testing.assert_allclose(camera.camera_to_world(points, rotations, offsets), expected)
    np.testing.assert_allclose(camera.world_to_camera(expected, rotations, offsets), points)

  def test_remove_root_and_intermediate(self):
    sk = Skeleton([-1, 0, 1, 1], [], [])
    self.assertEqual(sk.remove_joints([0, 1]), [2, 3])
    np.testing.assert_array_equal(sk.parents(), [-1, -1])

  def test_parent_after_child(self):
    sk = Skeleton([-1, 3, 0, 0], [], [])
    self.assertEqual(sk.remove_joints([2]), [0, 1, 3])
    np.testing.assert_array_equal(sk.parents(), [-1, 2, 0])

  def test_array_like_transform_inputs(self):
    rotation = transformation.rotation_matrix(np.pi, [0, 0, 1], [1, 0, 0])
    np.testing.assert_allclose(transformation.translation_from_matrix(rotation.tolist()), [2, 0, 0], atol=1e-12)
    self.assertAlmostEqual(transformation.angle_between_vectors([1, 0, 0], [0, 1, 0]), np.pi / 2)
