"""Utilities to convert imagery protobufs to other formats."""

import numpy as np

from pybullet_envs.minitaur.vision import imagery_pb2


# TODO(b/123306148): Support the conversion from image array to the proto.
def convert_image_to_array(image):
  """Converts an Image proto into a numpy array.

  Args:
    image: An instance of the imagery_pb2.Image proto.

  Returns:
    A numpy array. For color images (e.g. BGRA), the converted ND array
    has the format [Height, Width, Channel]. For gray images (e.g. depth), the
    converted ND array has the format [Height, Width].
  """

  if image.image_format == imagery_pb2.Image.IMAGE_FORMAT_BGRA_HWC_8U:
    img_buffer = np.fromstring(image.content, dtype=np.uint8)
    img = np.reshape(
        img_buffer, [image.height_px, image.width_px, 4], order="C")
    return img

  if image.image_format == imagery_pb2.Image.IMAGE_FORMAT_RGB_HWC_8U:
    img_buffer = np.fromstring(image.content, dtype=np.uint8)
    img = np.reshape(
        img_buffer, [image.height_px, image.width_px, 3], order="C")
    return img

  if image.image_format == imagery_pb2.Image.IMAGE_FORMAT_GRAY_HW_32F:
    img_buffer = np.fromstring(image.content, dtype=np.float32)
    img = np.reshape(img_buffer, [image.height_px, image.width_px], order="C")
    return img

  if image.image_format == imagery_pb2.Image.IMAGE_FORMAT_GRAY_HW_16U:
    img_buffer = np.fromstring(image.content, dtype=np.uint16)
    img = np.reshape(img_buffer, [image.height_px, image.width_px], order="C")
    return img

  raise ValueError("Unsupported image format {}".format(image.image_format))
