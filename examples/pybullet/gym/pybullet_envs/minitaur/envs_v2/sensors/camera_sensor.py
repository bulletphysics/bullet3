# Lint as: python3
"""A sensor for robot-mounted 1D lidar (laser scan)."""

from typing import Any, Iterable, Sequence, Type, Union

import gin
import gym
import numpy as np

from pybullet_envs.minitaur.envs_v2.sensors import sensor
from pybullet_envs.minitaur.vision import point_cloud_utils
from pybullet_envs.minitaur.vision import sim_camera

_MODE_TO_NUM_CHANNELS_DICT = {
    sim_camera.CameraMode.DEPTH: 1,
    sim_camera.CameraMode.RGB: 3,
    sim_camera.CameraMode.RGBD: 4,
    sim_camera.CameraMode.POINTCLOUD_WORLD_FRAME: 3,
    sim_camera.CameraMode.POINTCLOUD_ROBOT_FRAME: 3,
}


@gin.configurable
class CameraSensor(sensor.Sensor):
  """A robot-mounted sensor that returns RGBD images.

  Attributes:
    resolution: A 2-tuple (width, height) that represents the resolution of the
      camera.
    fov_degree: A floating point value that represents the field of view of the
      camera in the vertical direction. The unit is degree.
  """

  def __init__(self,
               camera_translation_from_base,
               camera_rotation_from_base,
               parent_link_id=-1,
               camera_mode=sim_camera.CameraMode.DEPTH,
               camera_update_frequency_hz=10,
               camera_stabilized=False,
               fov_degree=60,
               resolution=(32, 32),
               lower_bound: Union[float, Iterable[float]] = 0.0,
               upper_bound: Union[float, Iterable[float]] = 255.0,
               sensor_latency: Union[float, Sequence[float]] = 0.0,
               dtype: Type[Any] = np.float64,
               name="vision"):
    """Initializes the CameraSensor.

    Args:
      camera_translation_from_base: A 3-vector translation from the center of
        the specified link.
      camera_rotation_from_base: A 4-vector quaternion represents the rotation
        of the camera relative to the specified link.
      parent_link_id: The pybullet link id, where the camera is mounted on.
      camera_mode: An enum that specifies the mode that the camera operates. See
        sim_camera.CameraMode for more details.
      camera_update_frequency_hz: The frequency at which the camera will capture
        a frame.
      camera_stabilized: Whether the camera is stabilized. See
        sim_camera.MountedCamera for more details.
      fov_degree: The vertical field of view of the camera (in degree).
      resolution: A 2-tuple that represents the width and the height of the
        camera image.
      lower_bound: The lower bound of values of the camera output. It could be a
        single float or an array of floats with shape (height, width, channels).
      upper_bound: The upper bound of values of the camera output. It could be a
        single float or an array of floats with shape (height, width,
        channels)..
      sensor_latency: See base class.
      dtype: See base class.
      name: The name of the sensor.
    """
    super().__init__(
        name=name,
        sensor_latency=sensor_latency,
        interpolator_fn=sensor.closest_obs_blender)
    self._parent_link_id = parent_link_id
    self._camera_mode = camera_mode
    self._camera_translation_from_base = camera_translation_from_base
    self._camera_rotation_from_base = camera_rotation_from_base
    self.camera_update_frequency_hz = camera_update_frequency_hz
    self._time_interval_every_camera_update = (1.0 /
                                               self.camera_update_frequency_hz)
    self._camera_stabilized = camera_stabilized
    self._fov_degree = fov_degree
    self._resolution = resolution
    num_channels = _MODE_TO_NUM_CHANNELS_DICT[self._camera_mode]
    self._camera = None
    self._camera_image = None
    self._dtype = dtype
    if isinstance(lower_bound, list):
      lower_bound = np.array(lower_bound, dtype=self._dtype)
    else:
      lower_bound = lower_bound * np.ones(
          shape=(resolution[1], resolution[0], num_channels), dtype=self._dtype)
    if isinstance(upper_bound, list):
      upper_bound = np.array(upper_bound, dtype=self._dtype)
    else:
      upper_bound = upper_bound * np.ones(
          shape=(resolution[1], resolution[0], num_channels), dtype=self._dtype)
    self._observation_space = gym.spaces.Box(
        low=lower_bound, high=upper_bound, dtype=self._dtype)
    self._last_camera_image_timestamp = None

  def change_mounting_point(
      self,
      camera_translation_from_link: Sequence[float] = (0, 0, 0),
      camera_rotation_from_link: Sequence[float] = (0, 0, 0, 1),
      parent_link_id: int = -1):
    """Changes mounting point. Must be called before calls to set_robot().

    Args:
      camera_translation_from_link: A 3-vector translation from the center of
        the specified link.
      camera_rotation_from_link: A 4-vector quaternion represents the rotation
        of the camera relative to the specified link.
      parent_link_id: The pybullet link id, where the camera is mounted on.
    """
    self._parent_link_id = parent_link_id
    self._camera_translation_from_base = camera_translation_from_link
    self._camera_rotation_from_base = camera_rotation_from_link

  def set_robot(self, robot):
    super().set_robot(robot)

    self._camera = sim_camera.MountedCamera(
        pybullet_client=robot.pybullet_client,
        body_id=robot.robot_id,
        parent_link_id=self._parent_link_id,
        relative_translation=self._camera_translation_from_base,
        relative_rotation=self._camera_rotation_from_base,
        stabilized=self._camera_stabilized,
        camera_mode=self._camera_mode,
        fov_degree=self._fov_degree,
        resolution=self._resolution)

  def on_reset(self, env):
    self._env = env
    self._last_camera_image_timestamp = None
    super().on_reset(env)

  def _get_original_observation(self):
    if self._last_camera_image_timestamp is None or (
        self._robot.timestamp >= self._last_camera_image_timestamp +
        self._time_interval_every_camera_update):
      self._camera_image = self._camera.render_image().astype(self._dtype)
      self._last_camera_image_timestamp = self._robot.timestamp
    return self._robot.timestamp, self._camera_image

  def project_depth_map_to_point_cloud(self, depth_map, use_world_frame=True):
    """Convert the depth map into a 3D point cloud.

    Args:
      depth_map: A 2D numpy array with shape (height, width) which represents
        the depth map.
      use_world_frame: Whether converts the depth map into a point cloud in the
        world frame. If False, the point cloud is in the robot's local frame. If
        True, the point cloud is in the world frame if the robot's base
        position/orientation can be measured (e.g. in sim, using SLAM or mocap).

    Returns:
      A point cloud represented by a numpy array of shape (height, width, 3).
    """
    point_cloud = point_cloud_utils.distance_map_to_point_cloud(
        np.squeeze(depth_map), self.fov_degree / 180.0 * np.pi,
        depth_map.shape[1], depth_map.shape[0])
    if use_world_frame:
      point_cloud = (
          self._camera.transform_point_cloud_from_camera_to_world_frame(
              point_cloud))
    else:
      point_cloud = (
          self._camera.transform_point_cloud_from_camera_to_robot_frame(
              point_cloud))
    return point_cloud

  @property
  def resolution(self):
    return self._camera.resolution

  @property
  def fov_degree(self):
    return self._camera.fov_degree
