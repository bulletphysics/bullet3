"""Tests for imagery_utils."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import struct
import numpy as np

from pybullet_envs.minitaur.vision import imagery_pb2
from pybullet_envs.minitaur.vision import imagery_utils
from google3.testing.pybase import googletest


class ImageryUtilsTest(googletest.TestCase):

  def test_convert_bgra_images(self):
    image = imagery_pb2.Image(
        height_px=2,
        width_px=2,
        image_format=imagery_pb2.Image.IMAGE_FORMAT_BGRA_HWC_8U,
        content=b'ABCDABCDABCDABCD',
    )

    image_array = imagery_utils.convert_image_to_array(image)

    self.assertEqual(image_array.dtype, np.uint8)
    self.assertEqual(image_array.shape, (image.height_px, image.width_px, 4))
    self.assertEqual(image_array[0, 0, 0], ord('A'))
    self.assertEqual(image_array[1, 0, 3], ord('D'))

  def test_convert_rgb_images(self):
    image = imagery_pb2.Image(
        height_px=2,
        width_px=2,
        image_format=imagery_pb2.Image.IMAGE_FORMAT_RGB_HWC_8U,
        content=b'ABCABCABCABC',
    )

    image_array = imagery_utils.convert_image_to_array(image)

    self.assertEqual(image_array.dtype, np.uint8)
    self.assertEqual(image_array.shape, (image.height_px, image.width_px, 3))
    self.assertEqual(image_array[0, 0, 0], ord('A'))
    self.assertEqual(image_array[1, 1, 2], ord('C'))

  def test_convert_gray_32bit_images(self):
    image = imagery_pb2.Image(
        height_px=2,
        width_px=3,
        image_format=imagery_pb2.Image.IMAGE_FORMAT_GRAY_HW_32F,
        content=b'AAAABBBBCCCCAAAABBBBCCCC',
    )

    image_array = imagery_utils.convert_image_to_array(image)

    self.assertEqual(image_array.dtype, np.float32)
    self.assertEqual(image_array.shape, (image.height_px, image.width_px))
    self.assertEqual(image_array[0, 2], struct.unpack(b'<f', b'CCCC'))
    self.assertEqual(image_array[1, 1], struct.unpack(b'<f', b'BBBB'))

  def test_convert_gray_16bit_images(self):
    image = imagery_pb2.Image(
        height_px=3,
        width_px=2,
        image_format=imagery_pb2.Image.IMAGE_FORMAT_GRAY_HW_16U,
        content=b'AABBCCAABBCC',
    )

    image_array = imagery_utils.convert_image_to_array(image)

    self.assertEqual(image_array.dtype, np.uint16)
    self.assertEqual(image_array.shape, (image.height_px, image.width_px))
    self.assertEqual(image_array[0, 1], struct.unpack(b'<H', b'BB'))
    self.assertEqual(image_array[2, 1], struct.unpack(b'<H', b'CC'))

  def test_unspecified_image_format(self):
    image = imagery_pb2.Image()
    with self.assertRaises(ValueError):
      imagery_utils.convert_image_to_array(image)


if __name__ == '__main__':
  googletest.main()
