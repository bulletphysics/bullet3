"""Rendering utilities."""

import enum
import math
import os
from typing import Any, Callable, Dict, Iterable, Optional, Text
from absl import logging
import gin
import numpy as np


# These matrices will change by any call to render() and should be saved
# immediately to the local object.
last_used_view_matrix = None
last_used_proj_matrix = None
last_rendered_image_size = None

# Bounds of plane loaded with GEOM_PLANE.
_INFINITY = 1.0e30
# In case of infinite plane, use this scene bounding box.
_DEFAULT_BOUNDING_BOX = ((-15, -15, -10), (15, 15, 0))


def render_image(pybullet_client, width, height, view_matrix, proj_matrix):
  """Renders image as numpy array given view and projection matrices."""
  global last_used_view_matrix, last_used_proj_matrix, last_rendered_image_size
  last_used_view_matrix = view_matrix
  last_used_proj_matrix = proj_matrix

  (_, _, px, _, _) = pybullet_client.getCameraImage(
      width=width,
      height=height,
      renderer=pybullet_client.ER_BULLET_HARDWARE_OPENGL,
      viewMatrix=view_matrix,
      projectionMatrix=proj_matrix)
  rgb_array = np.array(px)
  image = rgb_array[:, :, :3]
  last_rendered_image_size = (image.shape[1], image.shape[0])
  return image


def project_world_to_image(points,
                           view_matrix=None,
                           proj_matrix=None,
                           image_size=None,
                           include_z_coord=False):
  """Projects 3D world-space points to 2D or 3D image-space points.

  If no projection matrices or image_size are given, the last ones are used.
  This means that you can render an image with `render_image` function (or other
  functions that use this one) and then project points to that image immediately
  after that call. Otherwise, you can save `last_used_view_matrix` and
  `last_used_proj_matrix` variables and pass them in later.

  Args:
    points: Sequence of 3D points.
    view_matrix: Optional view matrix (column-major).
    proj_matrix: Optional projection matrix (column-major).
    image_size: Optional image size for resulting coords.
    include_z_coord: Whether to include Z coordinate in the resulting
      projection. Z coordinate goes from 0 (far plane) to 1 (near plane).

  Returns:
    A Numpy array of shape (points_count, 2) or (points_count, 3) when
    `include_z_coord` is True, with dtype of np.float32.
  """
  # Note that these matrices are column-major.
  view_matrix = np.array(
      view_matrix or last_used_view_matrix, dtype=np.float32).reshape((4, 4))
  proj_matrix = np.array(
      proj_matrix or last_used_proj_matrix, dtype=np.float32).reshape((4, 4))
  mvp_matrix = np.matmul(view_matrix, proj_matrix)

  # Add w component equal to 1 (for perspective projection).
  points = np.asarray(points, dtype=np.float32)
  points_4d = np.concatenate(
      [points, np.ones((points.shape[0], 1), dtype=np.float32)], axis=-1)
  points_proj = np.matmul(points_4d, mvp_matrix)

  if include_z_coord:
    # Perspective divide (only keep X, Y, and Z, discard W).
    points_proj_3d = points_proj[:, 0:3] / np.expand_dims(points_proj[:, 3], -1)
    # Shift origin to bottom left and rescale range to [0,1]. This assumes
    # OpenGL projection space.
    points_proj_3d = (points_proj_3d + 1) * 0.5
    # Invert y-axis to have a coordinate with (0,0) on the top left.
    points_proj_3d[:, 1] = 1 - points_proj_3d[:, 1]
    # Scale projection to image size, ignore Z.
    image_size = np.asarray(
        image_size or last_rendered_image_size, dtype=np.float32)
    image_size = np.append(image_size, 1)
    return points_proj_3d * image_size
  else:
    # Perspective divide (only keep X and Y, discard Z and W).
    points_proj_2d = points_proj[:, 0:2] / np.expand_dims(points_proj[:, 3], -1)
    # Shift origin to bottom left and rescale range to [0,1]. This assumes
    # OpenGL projection space.
    points_proj_2d = (points_proj_2d + 1) * 0.5
    # Invert y-axis to have a coordinate with (0,0) on the top left.
    points_proj_2d[:, 1] = 1 - points_proj_2d[:, 1]
    # Scale projection to image size.
    image_size = np.asarray(
        image_size or last_rendered_image_size, dtype=np.float32)
    return points_proj_2d * image_size


def get_scene_bounding_box(pybullet_client, scene=None):
  """Computes scene axis-aligned bounding box.

  Args:
    pybullet_client: PyBullet client.
    scene: Scene instance for filtering the bounding box of camera.

  Returns:
    A tuple of min and max 3D coordinates. Returns (None, None) if the scene
    is empty.
  """
  aabb_min = None
  aabb_max = None

  for i in range(pybullet_client.getNumBodies()):
    body_id = pybullet_client.getBodyUniqueId(i)

    # If a scene has been provided, only count bodes which are in
    # either the ground or obstacle id lists.
    if scene is not None:
      if body_id not in scene.ground_ids and body_id not in scene.obstacle_ids:
        continue

    aabb = pybullet_client.getAABB(body_id)
    if np.any(np.abs(aabb) >= _INFINITY):
      aabb = _DEFAULT_BOUNDING_BOX
    if aabb_min is None:
      aabb_min = aabb[0]
      aabb_max = aabb[1]
    else:
      aabb_min = np.minimum(aabb_min, aabb[0])
      aabb_max = np.maximum(aabb_max, aabb[1])

  return aabb_min, aabb_max


@gin.configurable
def render_topdown(
    pybullet_client,
    result_size=(1280, 720),
    scale_px_per_meter=None,
    camera_height=50,
    ground_height=None,
    low_render_height_from_ground=None,
    high_render_height_from_ground=None,
    scene=None,
    use_y_as_up_axis=False,
    rendered_origin_and_size=None,
):
  """Renders top-down image of the environment.

  Args:
    pybullet_client: PyBullet client.
    result_size: Resulting image size.
    scale_px_per_meter: Resulting image scale in pixels per meter. This
      overrides `result_size`.
    camera_height: Height of the camera above the environment. This is not very
      significant, the lower the height, the larger perspective distortion.
    ground_height: Ground height for following two parameters.
    low_render_height_from_ground: If set, rendering is cut below this distance
      from ground.
    high_render_height_from_ground:  If set, rendering is cut above this
      distance from ground.
    scene: Scene instance for filtering the bounding box of camera.
    use_y_as_up_axis: Whether to consider Y axis as world's "up" instead of Z.
    rendered_origin_and_size: If set, sets rendered origin (bounding box min)
      and size (bounding box size) instead of computing that from the scene.
      Expected format: two tuples of [x, y z] coords containing origin and size
        of rendered bounding box.

  Returns:
    RGB image as 3D numpy array.
  """
  # Get bounds of the current environment.
  if rendered_origin_and_size is not None:
    aabb_min, aabb_size = rendered_origin_and_size
    if len(aabb_min) != 3:
      raise ValueError(
          "Invalid render origin, expected [x, y, z]: {}".format(aabb_min))
    if len(aabb_size) != 3:
      raise ValueError(
          "Invalid render size, expected [x, y, z]: {}".format(aabb_size))
    aabb_max = tuple(m + s for m, s in zip(aabb_min, aabb_size))
  else:
    aabb_min, aabb_max = get_scene_bounding_box(pybullet_client, scene)

  if use_y_as_up_axis:
    width = aabb_max[0] - aabb_min[0]
    height = aabb_max[2] - aabb_min[2]
    z_size = aabb_max[1] - aabb_min[1]
    z_max = aabb_max[1]
  else:
    width = aabb_max[0] - aabb_min[0]
    height = aabb_max[1] - aabb_min[1]
    z_size = aabb_max[2] - aabb_min[2]
    z_max = aabb_max[2]

  if scale_px_per_meter is not None:
    result_size = (int(width * scale_px_per_meter),
                   int(height * scale_px_per_meter))
  else:
    # Adjust scene size to fit inside of the given result size.
    if len(result_size) != 2 or result_size[0] <= 0 or result_size[1] <= 0:
      raise ValueError("Invalid result size: {}".format(result_size))
    img_width, img_height = result_size
    if img_width / width < img_height / height:
      # Width is limiting. Adjust height to match.
      adjusted_height = img_height * width / img_width
      assert adjusted_height >= height, (adjusted_height, height)
      height = adjusted_height
    else:
      # Height is limiting. Adjust width to match.
      adjusted_width = img_width * height / img_height
      assert adjusted_width >= width, (adjusted_width, width)
      width = adjusted_width

  # Compute view and projection matrices.
  if use_y_as_up_axis:
    center_x = (aabb_min[0] + aabb_max[0]) / 2.0
    center_z = (aabb_min[2] + aabb_max[2]) / 2.0
    view_matrix = pybullet_client.computeViewMatrix(
        cameraEyePosition=(center_x, aabb_max[1] + camera_height, center_z),
        cameraTargetPosition=(center_x, aabb_max[1], center_z),
        cameraUpVector=(0, 0, 1))
  else:
    center_x = (aabb_min[0] + aabb_max[0]) / 2.0
    center_y = (aabb_min[1] + aabb_max[1]) / 2.0
    view_matrix = pybullet_client.computeViewMatrix(
        cameraEyePosition=(center_x, center_y, aabb_max[2] + camera_height),
        cameraTargetPosition=(center_x, center_y, aabb_max[2]),
        cameraUpVector=(0, 1, 0))

  near_plane = camera_height
  far_plane = camera_height + z_size
  if ground_height is not None:
    if low_render_height_from_ground is not None:
      far_plane = (
          camera_height + z_max - ground_height - low_render_height_from_ground)
    if high_render_height_from_ground is not None:
      near_plane = (
          camera_height + z_max - ground_height -
          high_render_height_from_ground)
  else:
    if (low_render_height_from_ground is not None or
        high_render_height_from_ground is not None):
      raise ValueError(
          "The `low_render_height_from_ground` or "
          "`high_render_height_from_ground` were specified but no reference "
          "ground height was given.")
  vertical_fov = 2 * math.atan2(height / 2, camera_height)
  proj_matrix = pybullet_client.computeProjectionMatrixFOV(
      fov=math.degrees(vertical_fov),
      aspect=width / height,
      nearVal=near_plane,
      farVal=far_plane)

  # Render and return image.
  return render_image(
      pybullet_client,
      result_size[0],
      result_size[1],
      view_matrix,
      proj_matrix,
  )


