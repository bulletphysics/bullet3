"""Contains APIs to create in sim camera images."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import logging
import enum
import gin
import numpy as np
from pybullet_envs.minitaur.vision import point_cloud_utils

_DEFAULT_TARGET_DISTANCE = 10
_DEFAULT_FOV = 60
_DEFAULT_OPENGL_FRUSTUM_FAR = 100.0
_DEFAULT_OPENGL_FRUSTUM_NEAR = 0.01
_PYBULLET_DEFAULT_PROJECTION_MATRIX = (1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                       0.0, 0.0, -1.0000200271606445, -1.0, 0.0,
                                       0.0, -0.02000020071864128, 0.0)
# In the camera space, x axis points right, y axis points down and z axis points
# front. In the world space, x is front, y is left, and z is up. The following
# transformation rotate the camera coordinate to the world coordinate.
_CAMERA_Z_TO_WORLD_Z = (0.5, -0.5, 0.5, -0.5)


@gin.constants_from_enum
class CameraMode(enum.Enum):
  """Different modes that the camera can operate in."""

  RGB = 1
  DEPTH = 2
  RGBD = 3
  # The point cloud in the world frame, where z-axis is the up direction, x-axis
  # is the front direction, and y axis is the left direction.
  POINTCLOUD_WORLD_FRAME = 4
  # The point cloud in the robot's camera frame, where z-axis is the front
  # direction, x-axis is right direction, and y-axis is the downwards
  # direction.
  POINTCLOUD_ROBOT_FRAME = 5


def from_opengl_depth_to_distance(depth, near, far):
  return far * near / (far - (far - near) * depth)


def camera_intrinsics_to_projection_matrix(camera_intrinsics,
                                           width,
                                           height,
                                           far=_DEFAULT_OPENGL_FRUSTUM_FAR,
                                           near=_DEFAULT_OPENGL_FRUSTUM_NEAR):
  """Converts the camera intrinsics to OpenGL projection matrix.

  Args:
    camera_intrinsics: A CameraIntrinsics proto.
    width: Integer. The image width in pixels.
    height: Integer. The image height in pixels.
    far: Float. The far clipping plane distance.
    near: Float. The near clipping plane distance.

  Returns:
    An OpengGL projection matrix.

  """
  # first column
  projection_mat = [2 * camera_intrinsics.focal_length_px.x / width, 0, 0, 0]
  # second column
  projection_mat.extend(
      [0, 2 * camera_intrinsics.focal_length_px.y / height, 0, 0])
  # third column
  projection_mat.extend([
      2 * camera_intrinsics.principal_point_px.x / width - 1,
      2 * camera_intrinsics.principal_point_px.y / height - 1,
      -(far + near) / (far - near), -1
  ])
  # last column
  projection_mat.extend([0, 0, 2 * far * near / (near - far), 0])

  return tuple(projection_mat)


def create_camera_image(pybullet_client,
                        camera_position,
                        camera_orientation,
                        resolution,
                        projection_mat,
                        egl_render=False):
  """Creates a simulated camera image.

  Args:
    pybullet_client: A pybullet client.
    camera_position: A list of three floats. The absolute location of the camera
      in the simulated world.
    camera_orientation: A list of four floats. The orientation of the camera in
      world frame, in quaternion.
    resolution: A list of two integers. The horizontal and vertical resolution
      of the camera image in pixels.
    projection_mat: A list of 16 floats. The OpenGL projection matrix, in row
      major.
    egl_render: Whether to use EGL to render the images.

  Returns:
    A tuple containing the image resolution and the array for the sythesized RGB
    camera image.
  """

  orientation_mat = pybullet_client.getMatrixFromQuaternion(camera_orientation)

  # The first column in the orientation matrix.
  forward_vec = orientation_mat[::3]
  target_distance = _DEFAULT_TARGET_DISTANCE
  camera_target = [
      camera_position[0] + forward_vec[0] * target_distance,
      camera_position[1] + forward_vec[1] * target_distance,
      camera_position[2] + forward_vec[2] * target_distance
  ]

  # The third column in the orientation matrix. We assume camera up vector is
  # always [0, 0, 1] in its local frame.
  up_vec = orientation_mat[2::3]

  view_mat = pybullet_client.computeViewMatrix(camera_position, camera_target,
                                               up_vec)
  renderer = (
      pybullet_client.ER_BULLET_HARDWARE_OPENGL
      if egl_render else pybullet_client.ER_TINY_RENDERER)
  return pybullet_client.getCameraImage(
      resolution[0],
      resolution[1],
      viewMatrix=view_mat,
      projectionMatrix=projection_mat,
      renderer=renderer)

class MountedCamera(object):
  """A camera that is fixed to a robot's body part.

  Attributes:
    resolution: A 2-tuple (width, height) that represents the resolution of the
      camera.
    fov_degree: A floating point value that represents the field of view of the
      camera in the vertical direction. The unit is degree.
  """

  def __init__(self,
               pybullet_client,
               body_id,
               parent_link_id,
               relative_translation,
               relative_rotation,
               resolution,
               fov_degree=_DEFAULT_FOV,
               depth_lower_limit=_DEFAULT_OPENGL_FRUSTUM_NEAR,
               depth_upper_limit=_DEFAULT_OPENGL_FRUSTUM_FAR,
               stabilized=False,
               camera_intrinsics=None,
               camera_mode=CameraMode.RGB,
               normalize_rgb=False,
               egl_render=False):
    """Initializes the MounteeCamera class.

    Args:
      pybullet_client: A BulletClient instance.
      body_id: Integer. The unique body ID returned from loadURDF in pybullet.
      parent_link_id: Integer. The camera is rigidly attached to a body link
        specified by this ID. For example, camers may be mounted to the base or
        other moving parts of the robot.
      relative_translation: A list of three floats. The relative translation
        between the center of the camera and the link.
      relative_rotation: A list of four floats. The quaternion specifying the
        relative rotation of the camera.
      resolution: A list of two integers.
      fov_degree: The vertical field of view of this camera. Has no effect if
        camera_intrinsics is provided.
      depth_lower_limit: The lower limit of the depth camera.
      depth_upper_limit: The upper limit of the depth camera.
      stabilized: Weather the camera's roll and pitch should be stabilized. If
        turned on, only yaw rotation is applied to the camera.
      camera_intrinsics: A CameraIntrinsics proto instance.
      camera_mode: The mode of the camera. It can be set in the following
        choices [CameraMode.RGB, CameraMode.DEPTH, CameraMode.RGBD].
      normalize_rgb: Whether to normalize the output image pixel values to [-1,
        1].
      egl_render: Whether to use EGL to render the images when the X11 windows
        are not present.
    """
    self._pybullet_client = pybullet_client
    self._body_id = body_id
    self._parent_link_id = parent_link_id
    self._relative_translation = relative_translation
    self._relative_rotation = relative_rotation
    self._resolution = resolution
    self._stabilized = stabilized
    self._camera_mode = camera_mode
    self._fov_degree = fov_degree
    self._near = depth_lower_limit
    self._far = depth_upper_limit
    self._egl_render = egl_render
    self._normalize_rgb = normalize_rgb
    self._prev_depth = None
    width, height = resolution
    if camera_intrinsics is not None:
      self._projection_mat = camera_intrinsics_to_projection_matrix(
          camera_intrinsics, width, height, near=self._near, far=self._far)
    else:
      self._projection_mat = self._pybullet_client.computeProjectionMatrixFOV(
          fov_degree,
          float(width) / float(height), self._near, self._far)

  def __call__(self):
    return self.render_image()

  def transform_point_cloud_from_camera_to_world_frame(self, point_cloud):
    """Project a sample in the depth image to a 3D point in the world frame.

    Args:
      point_cloud: A numpy array of shape (height, width, 3) that represents
        a point cloud in the camera frame.

    Returns:
      The transformed point cloud in the world frame.
    """
    if self._parent_link_id == -1:
      camera_link_pos, camera_link_ori = (
          self._pybullet_client.getBasePositionAndOrientation(self._body_id))
    else:
      parent_link_state = self._pybullet_client.getLinkState(
          self._body_id, self._parent_link_id, computeForwardKinematics=True)
      camera_link_pos = parent_link_state[0]
      camera_link_ori = parent_link_state[1]

    camera_position_world, camera_orientation_world = (
        self._pybullet_client.multiplyTransforms(camera_link_pos,
                                                 camera_link_ori,
                                                 self._relative_translation,
                                                 self._relative_rotation))
    _, camera_space_to_world_orientation = (
        self._pybullet_client.multiplyTransforms([0, 0, 0],
                                                 camera_orientation_world,
                                                 [0, 0, 0],
                                                 _CAMERA_Z_TO_WORLD_Z))
    for i in range(point_cloud.shape[0]):
      for j in range(point_cloud.shape[1]):
        point_cloud[i, j, :] = (
            self._pybullet_client.multiplyTransforms(
                camera_position_world, camera_space_to_world_orientation,
                point_cloud[i, j, :], [0, 0, 0, 1])[0])
    return point_cloud

  def transform_point_cloud_from_camera_to_robot_frame(self, point_cloud):
    """Project a sample in the depth image to a 3D point in the robot frame.

    Args:
      point_cloud: A numpy array of shape (height, width, 3) that represents
        a point cloud in the camera frame.

    Returns:
      The transformed point cloud in the robot frame.
    """
    camera_space_to_robot_translation, camera_space_to_robot_orientation = (
        self._pybullet_client.multiplyTransforms(self._relative_translation,
                                                 self._relative_rotation,
                                                 [0, 0, 0],
                                                 _CAMERA_Z_TO_WORLD_Z))
    for i in range(point_cloud.shape[0]):
      for j in range(point_cloud.shape[1]):
        point_cloud[i, j, :] = (
            self._pybullet_client.multiplyTransforms(
                camera_space_to_robot_translation,
                camera_space_to_robot_orientation, point_cloud[i, j, :],
                [0, 0, 0, 1])[0])
    return point_cloud

  def render_image(self):
    """Retrieves the camera image.

    Returns:
      A simulated view from the camera's perspective.
    """
    pos = []
    ori = []
    if self._parent_link_id == -1:
      pos, ori = self._pybullet_client.getBasePositionAndOrientation(
          self._body_id)
    else:
      parent_link_state = self._pybullet_client.getLinkState(
          self._body_id, self._parent_link_id, computeForwardKinematics=True)
      pos = parent_link_state[0]
      ori = parent_link_state[1]
    if self._stabilized:
      [_, _, yaw] = self._pybullet_client.getEulerFromQuaternion(ori)
      ori = self._pybullet_client.getQuaternionFromEuler([0, 0, yaw])

    transform = self._pybullet_client.multiplyTransforms(
        pos, ori, self._relative_translation, self._relative_rotation)

    _, _, rgba, depth, _ = create_camera_image(
        pybullet_client=self._pybullet_client,
        camera_position=transform[0],
        camera_orientation=transform[1],
        resolution=self._resolution,
        projection_mat=self._projection_mat,
        egl_render=self._egl_render)
    if np.any(np.isnan(depth)):
      logging.info("depth contained nan: %s, replacing with prev_depth: %s",
                   depth, self._prev_depth)
      depth = self._prev_depth
    self._prev_depth = depth
    rgba = np.reshape(np.array(rgba, dtype=np.float32), (self._resolution[0], self._resolution[1], -1))

    # Converts from OpenGL depth map to a distance map (unit: meter).
    distances = from_opengl_depth_to_distance(
        np.reshape(np.array(depth), (self._resolution[0], self._resolution[1], -1)), self._near, self._far)
    if self._normalize_rgb:
      # Does not normalize the depth image.
      rgba = 2 * rgba / 255.0 - 1

    if self._camera_mode == CameraMode.RGB:
      return rgba[:, :, 0:3]  # Remove the alpha channel
    elif self._camera_mode == CameraMode.DEPTH:
      return distances[:, :, np.newaxis]
    elif self._camera_mode == CameraMode.RGBD:
      rgbd_image = rgba
      rgbd_image[:, :, 3] = distances
      return rgbd_image
    elif self._camera_mode == CameraMode.POINTCLOUD_WORLD_FRAME:
      point_cloud = point_cloud_utils.distance_map_to_point_cloud(
          distances, self._fov_degree / 180.0 * np.pi, distances.shape[1],
          distances.shape[0])
      point_cloud = self.transform_point_cloud_from_camera_to_world_frame(
          point_cloud)
      return point_cloud
    elif self._camera_mode == CameraMode.POINTCLOUD_ROBOT_FRAME:
      point_cloud = point_cloud_utils.distance_map_to_point_cloud(
          distances, self._fov_degree / 180.0 * np.pi, distances.shape[1],
          distances.shape[0])
      point_cloud = self.transform_point_cloud_from_camera_to_robot_frame(
          point_cloud)
      return point_cloud
    else:
      raise NotImplementedError("Camera mode {} is not implemented.".format(
          self._camera_mode))

  @property
  def resolution(self):
    return self._resolution

  @property
  def fov_degree(self):
    return self._fov_degree
