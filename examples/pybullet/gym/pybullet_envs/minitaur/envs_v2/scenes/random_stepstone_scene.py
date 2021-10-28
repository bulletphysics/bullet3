# Lint as: python3
"""Scene with randomly spaced stepstones."""

from typing import Optional, Sequence

import gin
import numpy as np

from pybullet_envs.minitaur.envs_v2.scenes import scene_base
from pybullet_envs.minitaur.envs_v2.scenes.terrain import stepstones


@gin.configurable
class RandomStepstoneScene(scene_base.SceneBase):
  """Scene with randomly spaced stepstones."""

  def __init__(
      self,
      num_stones: int = 50,
      stone_height: float = 0.1,
      stone_width_lower_bound: float = 10.0,
      stone_width_upper_bound: float = 10.0,
      stone_length_lower_bound: float = 0.1,
      stone_length_upper_bound: float = 0.3,
      gap_length_lower_bound: float = 0.1,
      gap_length_upper_bound: float = 0.3,
      height_offset_lower_bound: float = 0.0,
      height_offset_upper_bound: float = 1e-6,
      floor_height_lower_bound: float = -0.5,
      floor_height_upper_bound: float = -0.55,
      platform_length_lower_bound: float = 0.75,
      platform_length_upper_bound: float = 1.0,
      random_seed: Optional[int] = None,
      color_sequence: Sequence[Sequence[float]] = stepstones.MULTICOLOR,
      rebuild_scene_during_reset: bool = True):
    """Initializes RandomStepstoneScene.

    Args:
      num_stones: The number of stepstones.
      stone_height: The height in meters of each stepstone.
      stone_width_lower_bound: The lower bound in meters of the randomly sampled
        stepstone width.
      stone_width_upper_bound: The upper bound in meters of the randomly sampled
        stepstone width.
      stone_length_lower_bound: The lower bound in meters of the randomly
        sampled stepstone length.
      stone_length_upper_bound: The upper bound in meters of the randomly
        sampled stepstone length.
      gap_length_lower_bound: The lower bound in meters of the random sampled
        gap distance.
      gap_length_upper_bound: The upper bound in meters of the random sampled
        gap distance.
      height_offset_lower_bound: The lower bound in meters of the randomly
        sampled stepstone height.
      height_offset_upper_bound: The upper bound in meters of the randomly
        sampled stepstone height.
      floor_height_lower_bound: The lower bound in meters of the randomly
        sampled floor height.
      floor_height_upper_bound: The upper bound in meters of the randomly
        sampled floor height.
      platform_length_lower_bound: The lower bound in meters of the first step
        stone length.
      platform_length_upper_bound: The upper bound in meters of the first step
        stone length.
      random_seed: The random seed to generate the random stepstones.
      color_sequence: A list of (red, green, blue, alpha) colors where each
        element is in [0, 1] and alpha is transparency. The stepstones will
        cycle through these colors.
      rebuild_scene_during_reset: Whether to rebuild the stepstones during
        reset.
    """
    for color in color_sequence:
      if len(color) != 4:
        raise ValueError(
            "Each color must be length 4; got <{}>".format(color_sequence))

    super(RandomStepstoneScene, self).__init__(data_root=None)
    self._num_stones = num_stones
    self._stone_height = stone_height
    self._stone_width_lower_bound = stone_width_lower_bound
    self._stone_width_upper_bound = stone_width_upper_bound
    self._stone_length_lower_bound = stone_length_lower_bound
    self._stone_length_upper_bound = stone_length_upper_bound
    self._gap_length_lower_bound = gap_length_lower_bound
    self._gap_length_upper_bound = gap_length_upper_bound
    self._height_offset_lower_bound = height_offset_lower_bound
    self._height_offset_upper_bound = height_offset_upper_bound
    self._floor_height_lower_bound = floor_height_lower_bound
    self._floor_height_upper_bound = floor_height_upper_bound
    self._platform_length_lower_bound = platform_length_lower_bound
    self._platform_length_upper_bound = platform_length_upper_bound
    self._random_seed = random_seed
    self._color_sequence = color_sequence
    self._rebuild_scene_during_reset = rebuild_scene_during_reset

  def reset(self):
    super().reset()

    if self._rebuild_scene_during_reset:
      for ground_id in self.ground_ids:
        self._pybullet_client.removeBody(ground_id)
      self.build_scene(self._pybullet_client)

  def build_scene(self, pybullet_client):
    super().build_scene(pybullet_client)
    # The first stone is to let the robot stand at the initial position.

    stone_width = np.random.uniform(self._stone_width_lower_bound,
                                    self._stone_width_upper_bound)
    platform_length = np.random.uniform(self._platform_length_lower_bound,
                                        self._platform_length_upper_bound)

    end_pos, first_stone_id = stepstones.build_one_stepstone(
        pybullet_client=pybullet_client,
        start_pos=(-platform_length / 2.0, 0, 0),
        stone_length=platform_length,
        stone_height=self._stone_height,
        stone_width=stone_width,
        gap_length=0.0,
        height_offset=0.0,
        rgba_color=stepstones.GRAY)

    _, stepstone_ids = stepstones.build_random_stepstones(
        pybullet_client=self.pybullet_client,
        start_pos=end_pos,
        num_stones=self._num_stones,
        stone_height=self._stone_height,
        stone_width=stone_width,
        stone_length_lower_bound=self._stone_length_lower_bound,
        stone_length_upper_bound=self._stone_length_upper_bound,
        gap_length_lower_bound=self._gap_length_lower_bound,
        gap_length_upper_bound=self._gap_length_upper_bound,
        height_offset_lower_bound=self._height_offset_lower_bound,
        height_offset_upper_bound=self._height_offset_upper_bound,
        random_seed=self._random_seed,
        color_sequence=self._color_sequence)
    for pybullet_id in [first_stone_id] + stepstone_ids:
      self.add_object(pybullet_id, scene_base.ObjectType.GROUND)

    self._floor_height = np.random.uniform(self._floor_height_lower_bound,
                                           self._floor_height_upper_bound)
    # Add floor
    floor_id = stepstones.load_box(pybullet_client,
        half_extents=[100, 100, 1],
        position=np.array([0.0, 0.0, self._floor_height - 1.0]),
        orientation=(0.0, 0.0, 0.0, 1.0),
        rgba_color=(1.0, 1.0, 1.0, 1.0),
        mass=0)
    self.add_object(floor_id, scene_base.ObjectType.GROUND)
