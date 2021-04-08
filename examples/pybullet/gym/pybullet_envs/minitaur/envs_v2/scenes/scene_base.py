# Lint as: python3
"""Class for loading and managing a scene in pybullet."""

import enum
from typing import Any, Dict, List, Optional, Sequence, Text
import gin
import numpy as np

from pybullet_envs.minitaur.envs_v2.scenes import world_asset_pb2
from pybullet_envs.minitaur.envs_v2 import base_client
from pybullet_utils import bullet_client
from pybullet_envs.minitaur.robots import autonomous_object



# The 2D coordinates of the corners of a polygon. The corners are specified in
# counterclock-wise direction.
Polygon = Sequence[Sequence[float]]


class ObjectType(enum.Enum):
  """Categories of objects that may be found in a scene."""
  OTHER = 0
  GROUND = 1
  OBSTACLE = 2
  GOAL = 3
  DYNAMIC_OBJECT = 4


@gin.configurable
class SceneBase(object):
  """Class for loading and managing a scene."""

  def __init__(
      self,
      data_root: Text = None,
      dynamic_objects: Sequence[autonomous_object.AutonomousObject] = ()):
    """Initializes SceneBase.

    Args:
      data_root: Root directory for finding object models.
      dynamic_objects: Dynamic objects to be added into the scene.
      crowd_builders: Builders of crowds formed by autonomous objects.
    """
    self._pybullet_client = None
    self._data_root = data_root
    temp_dynamic_objects = list(dynamic_objects)
    
    self._dynamic_objects = tuple(temp_dynamic_objects)
    # Dictionaries and world_asset are declared outside init to make sure they
    # are all reset in _reset_scene_tracking().
    self._reset_scene_tracking()

  def _reset_scene_tracking(self):
    """Clears all scene dicts. Used when the simulation is reset."""
    self._type_to_ids_dict = {object_type: [] for object_type in ObjectType}
    self._type_to_objects_dict = {object_type: [] for object_type in ObjectType}
    self._id_to_type_dict = {}
    self._id_to_object_dict = {}
    self._world_asset = None

  def destroy_scene(self):
    """Destroys contents of scene to get ready for another build_scene call."""
    id_to_remove = list(self._id_to_object_dict.keys())
    for object_id in id_to_remove:
      self.remove_object(object_id)
    self._reset_scene_tracking()

  def build_scene(self, pybullet_client: bullet_client.BulletClient):
    """Loads and positions all scene objects in pybullet.

    Override this function in subclass to implement customized scene. The
    overriding function must call base function first.

    Args:
      pybullet_client: A pybullet client in which the scene will be built.
    """
    self._reset_scene_tracking()
    self._pybullet_client = pybullet_client
    self._init_dynamic_objects()

  def reset(self):
    """The soft reset of scene.

    Unlike "build_scene", this is called at each env.reset() before robot
    resetting. Typically we use this API to do some soft resetting like putting
    objects back to its place. Howevever, for special cases such as P2P multimap
    training, we can reload a different mesh scene once a while.
    """
    pass
    
  def _init_dynamic_objects(self):
    """Adds dynamic objects to scene."""
    for an_object in self._dynamic_objects:
      an_object.set_sim_client(self._pybullet_client)
      self.add_object(an_object.sim_object_id, ObjectType.DYNAMIC_OBJECT,
                      an_object)

  @property
  def pybullet_client(self) -> bullet_client.BulletClient:
    if self._pybullet_client is None:
      raise ValueError("pybullet_client is None; did you call build_scene()?")
    return self._pybullet_client

  @property
  def ground_height(self) -> float:
    """Returns ground height of the scene."""
    return 0.0

  @property
  def ground_ids(self) -> List[int]:
    """Returns the pybullet ids of the ground."""
    return self._type_to_ids_dict[ObjectType.GROUND]

  @property
  def obstacle_ids(self) -> List[int]:
    """Returns the pybullet ids of all obstacles in the scene."""
    return self._type_to_ids_dict[ObjectType.OBSTACLE]

  @property
  def goal_ids(self) -> List[int]:
    """Returns the pybullet ids of any goals in the scene."""
    return self._type_to_ids_dict[ObjectType.GOAL]

  @property
  def dynamic_object_ids(self) -> List[int]:
    """Returns the pybullet ids of dynamic objects."""
    return self._type_to_ids_dict[ObjectType.DYNAMIC_OBJECT]

  @property
  def dynamic_objects(self) -> List[autonomous_object.AutonomousObject]:
    """Returns the dynamic objects python object (AutonomousObject)."""
    return self._type_to_objects_dict[ObjectType.DYNAMIC_OBJECT]

  @property
  def world_asset(self) -> world_asset_pb2.WorldAsset:
    """Returns a proto describing the semantics of the scene.

    If the scene keeps a WorldAsset, then mutating this proto will mutate it for
    everyone. If the scene generates a WorldAsset from _type_to_ids_dict, then
    this is not an issue.
    """
    if self._world_asset:
      return self._world_asset
    return self._dict_to_world_asset(self._type_to_ids_dict)

  def add_object(self,
                 object_id: int,
                 class_label: ObjectType,
                 python_object: Optional[Any] = None):
    """Adds an object to be tracked.

    Does not load anything into pybullet.

    Args:
      object_id: objectUniqueId from pybullet.
      class_label: What type to consider the new object.
      python_object: Associated python object for the pybullet object of
        objectUniqueId == object_id. Environment uses the python object to
        control object in pybullet in these cases. One example is python objects
        of class label DYNAMIC_OBJECT: they are associated with python objects
          of type AutonomousObject.
    """
    if python_object is not None:
      if (isinstance(python_object, autonomous_object.AutonomousObject) and
          python_object.sim_object_id != object_id):
        raise ValueError(
            f"Mismatch object ids, object_id = {object_id}, sim_object_id = "
            f"{python_object.sim_object_id}")
      self._type_to_objects_dict[class_label].append(python_object)
    self._type_to_ids_dict[class_label].append(object_id)
    self._id_to_type_dict[object_id] = class_label
    self._id_to_object_dict[object_id] = python_object

  def remove_object(self, object_id: int):
    """Removes an object from tracking and from pybullet.

    Args:
      object_id: objectUniqueID from pybullet.

    Raises:
      KeyError: if object_id does not exist in the record.
    """
    if object_id not in self._id_to_type_dict.keys():
      raise KeyError(
          f"Object with object_id = {object_id} does not exist in the record.")

    self.pybullet_client.removeBody(object_id)
    object_type = self._id_to_type_dict[object_id]
    self._type_to_ids_dict[object_type].remove(object_id)

    object_to_remove = self.id_to_object(object_id)
    if object_to_remove is not None:
      # Removes item by identity comparison and avoid slow down due to objects
      # with complex equality comparison function. list.remove() compares
      # equality instead of identity.
      for i, an_object in enumerate(self._type_to_objects_dict[object_type]):
        if an_object is object_to_remove:
          del self._type_to_objects_dict[object_type][i]
          break
    del self._id_to_type_dict[object_id]
    del self._id_to_object_dict[object_id]

  def id_to_object(self, object_id: int) -> Any:
    """Returns underlying python object from sim object id.

    Args:
      object_id: objectUniqueID from pybullet.

    Returns:
      None is returned if the sim object does not have a corresponding python
      object.
    """
    return self._id_to_object_dict[object_id]

  def _dict_to_world_asset(
      self, type_to_ids_dict: Dict[ObjectType,
                                   List[int]]) -> world_asset_pb2.WorldAsset:
    """Converts a dictionary to a WorldAsset.

    Args:
      type_to_ids_dict: Dictionary that describes the scene. Keys are
        ObjectTypes and values are lists of integers, where each integer is a
        pybullet id for an object of a given type.

    Returns:
      A WorldAsset proto with the types, locations and bounding boxes of all
      objects in the scene.
    """
    world_asset = world_asset_pb2.WorldAsset()
    for object_type in type_to_ids_dict.keys():
      for obj_id in type_to_ids_dict[object_type]:
        bbox = np.array(self.pybullet_client.getAABB(obj_id))
        bbox_center = np.mean(bbox, axis=0)
        bbox_dimensions = bbox[1] - bbox[0]

        obj = world_asset_pb2.Object()
        obj.id = str(obj_id)
        obj.label = str(object_type)
        obj.bounding_box.center.x = bbox_center[0]
        obj.bounding_box.center.y = bbox_center[1]
        obj.bounding_box.center.z = bbox_center[2]
        obj.bounding_box.dimensions.x = bbox_dimensions[0]
        obj.bounding_box.dimensions.y = bbox_dimensions[1]
        obj.bounding_box.dimensions.z = bbox_dimensions[2]
        world_asset.objects.append(obj)
    return world_asset

  def close(self):
    """Closes the scene at the end of life cycle of the environment."""
    pass

  @property
  def vectorized_map(self) -> Sequence[Polygon]:
    """Returns vectorized map containing a list of polygon obstacles."""
    raise NotImplementedError("vectorized_map is not implemented by default.")
