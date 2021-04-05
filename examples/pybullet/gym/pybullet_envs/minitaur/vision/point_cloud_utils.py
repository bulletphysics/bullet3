# Lint as: python3
"""Common functionalities to process point clouds."""

import math
import numpy as np


def distance_map_to_point_cloud(distances, fov, width, height):
  """Converts from a depth map to a point cloud.

  Args:
    distances: An numpy array which has the shape of (height, width) that
      denotes a distance map. The unit is meter.
    fov: The field of view of the camera in the vertical direction. The unit
      is radian.
    width: The width of the image resolution of the camera.
    height: The height of the image resolution of the camera.

  Returns:
    point_cloud: The converted point cloud from the distance map. It is a numpy
      array of shape (height, width, 3).
  """
  f = height / (2 * math.tan(fov / 2.0))
  px = np.tile(np.arange(width), [height, 1])
  x = (2 * (px + 0.5) - width) / f * distances / 2
  py = np.tile(np.arange(height), [width, 1]).T
  y = (2 * (py + 0.5) - height) / f * distances / 2
  point_cloud = np.stack((x, y, distances), axis=-1)
  return point_cloud


def point_cloud_normals(points):
  """Estimates the normals of a point cloud using cross product.

  Args:
    points: a point cloud represented as an 2D array of which has the shape of
      [m, n, 3].

  Returns:
    normals: a 2D array of normal vectors which has the same shape as the input.
  """
  grad = np.gradient(points, axis=(0, 1))  # calculates gradient along x and y
  normal = np.cross(grad[0], grad[1])
  norms = np.linalg.norm(normal, axis=2)[:, :, None]  # norms in shape [m,n,1]
  normalized = normal / (norms + 1e-7)  # adds eps to avoid div by 0
  return normalized


def point_cloud_distances(point, point_cloud):
  """Estimates the distance of a point to every point in a point cloud.

  Args:
    point: a 3D vector representing a point.
    point_cloud: a point cloud represented as an 2D array of which has the shape
      of [m, n, 3].

  Returns:
    distances: a distance field represented as a 2D array [m, n].
  """
  return np.linalg.norm(point_cloud - point, axis=2)
