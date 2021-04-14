# Lint as: python3
"""A scene containing only a planar floor."""

from typing import Sequence

import gin
from pybullet_envs.minitaur.envs_v2 import base_client
from pybullet_envs.minitaur.envs_v2.scenes import scene_base

_PLANE_URDF = (
    "plane.urdf")


@gin.configurable
class SimpleScene(scene_base.SceneBase):
  """A scene containing only a planar floor."""

  def build_scene(self, pybullet_client):
    super().build_scene(pybullet_client)

    visual_shape_id = self._pybullet_client.createVisualShape(
        shapeType=self._pybullet_client.GEOM_PLANE)
    collision_shape_id = self._pybullet_client.createCollisionShape(
        shapeType=self._pybullet_client.GEOM_PLANE)
    ground_id = self._pybullet_client.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision_shape_id,
        baseVisualShapeIndex=visual_shape_id)
    self._pybullet_client.changeDynamics(ground_id, -1, lateralFriction=1.0)
    self.add_object(ground_id, scene_base.ObjectType.GROUND)

  @property
  def vectorized_map(self) -> Sequence[scene_base.Polygon]:
    """Returns vectorized map containing a list of polygon obstacles."""
    return []
