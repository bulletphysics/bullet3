# Lint as: python3
"""Generates stepstones in PyBullet simulation."""

import itertools
import random
from typing import List, Optional, Sequence, Tuple
import numpy as np
import pybullet
from pybullet_utils import bullet_client

GRAY = (0.3, 0.3, 0.3, 1)
# RGB values from the official Google logo colors.
GREEN = [60/255, 186/255, 84/255, 1]
YELLOW = [244/255, 194/255, 13/255, 1]
RED = [219/255, 50/255, 54/255, 1]
BLUE = [72/255, 133/255, 237/255, 1]

MULTICOLOR = (GREEN, YELLOW, RED, BLUE)

def load_box(pybullet_client: bullet_client.BulletClient,
               half_extents: Sequence[float] = (1, 1, 1),
               position: Sequence[float] = (0, 0, 0),
               orientation: Sequence[float] = (0, 0, 0, 1),
               rgba_color: Sequence[float] = (0.3, 0.3, 0.3, 1),
               mass: float = 0) -> int:
    """Loads a visible and tangible box.

    Args:
      half_extents: Half the dimension of the box in meters in each direction.
      position: Global coordinates of the center of the box.
      orientation: As a quaternion.
      rgba_color: The color and transparency of the box, each in the range
        [0,1]. Defaults to opaque gray.
      mass: Mass in kg. Mass 0 fixes the box in place.

    Returns:
      Unique integer referring to the loaded box.
    """
    col_box_id = pybullet_client.createCollisionShape(
        pybullet.GEOM_BOX, halfExtents=half_extents)
    visual_box_id = pybullet_client.createVisualShape(
        pybullet.GEOM_BOX, halfExtents=half_extents, rgbaColor=rgba_color)
    return pybullet_client.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=col_box_id,
        baseVisualShapeIndex=visual_box_id,
        basePosition=position,
        baseOrientation=orientation)


def build_one_stepstone(
    pybullet_client: bullet_client.BulletClient,
    start_pos: Sequence[float] = (0, 0, 0),
    stone_length: float = 1,
    stone_height: float = 0.15,
    stone_width: float = 5.0,
    gap_length: float = 0.3,
    height_offset: float = 0,
    rgba_color: Sequence[float] = GRAY) -> Tuple[np.ndarray, int]:
  """Generates one stepstone.

  Args:
    pybullet_client: The pybullet client instance.
    start_pos: The starting position (the midpoint of top-left edge) of the
      stepstone.
    stone_length: The length of the stepstone in meters.
    stone_height: The height of the stepstone in meters.
    stone_width: The width of the stepstone in meters.
    gap_length: The distance in meters between two adjacent stepstones.
    height_offset: The height difference in meters between two adjacent
      stepstones.
    rgba_color: The color and transparency of the object, each in the range
      [0,1]. Defaults to opaque gray.

  Returns:
    The position of the mid point of the right-top edge of the stepstone.
    The pybullet id of the stepstone.
  """
  half_length = stone_length / 2.0
  half_width = stone_width / 2.0
  half_height = stone_height / 2.0
  start_pos = np.asarray(start_pos) + np.array([gap_length, 0, height_offset])
  step_stone_id = load_box(pybullet_client,
      half_extents=[half_length, half_width, half_height],
      position=start_pos + np.array([half_length, 0, -half_height]),
      orientation=(0, 0, 0, 1),
      rgba_color=rgba_color,
      mass=0)
  end_pos = start_pos + np.array([stone_length, 0, 0])
  return end_pos, step_stone_id


def build_stepstones(
    pybullet_client: bullet_client.BulletClient,
    start_pos: Sequence[float] = (0, 0, 0),
    num_stones: int = 5,
    stone_length: float = 1,
    stone_height: float = 0.15,
    stone_width: float = 5.0,
    gap_length: float = 0.3,
    color_sequence: Sequence[Sequence[float]] = MULTICOLOR
) -> Tuple[np.ndarray, List[int]]:
  """Generates a series of stepstones.

  All the stepstones share the same parameters, such as length, height, width
  and gap distance.

  Args:
    pybullet_client: The pybullet client instance.
    start_pos: The starting position (the mid point of top-left edge) of the
      first stepstone.
    num_stones: The number of stepstones in this creation.
    stone_length: The length of the stepstones in meters.
    stone_height: The height of the stepstones in meters.
    stone_width: The width of the stepstones in meters.
    gap_length: The distance in meters between two adjacent stepstones.
    color_sequence: A list of (red, green, blue, alpha) colors where each
      element is in [0, 1] and alpha is transparency. The stepstones will cycle
      through these colors.

  Returns:
    The position of the midpoint of the right-top edge of the last stepstone.
    The pybullet ids of the stepstones.
  """
  end_pos = start_pos
  ids = []
  for _, color in zip(range(num_stones), itertools.cycle(color_sequence)):
    end_pos, step_stone_id = build_one_stepstone(
        pybullet_client=pybullet_client,
        start_pos=end_pos,
        stone_length=stone_length,
        stone_height=stone_height,
        stone_width=stone_width,
        gap_length=gap_length,
        height_offset=0,
        rgba_color=color)
    ids.append(step_stone_id)
  return end_pos, ids


def build_random_stepstones(
    pybullet_client: bullet_client.BulletClient,
    start_pos: Sequence[float] = (0, 0, 0),
    num_stones: int = 5,
    stone_height: float = 0.15,
    stone_width: float = 5.0,
    stone_length_lower_bound: float = 0.5,
    stone_length_upper_bound: float = 1.5,
    gap_length_lower_bound: float = 0,
    gap_length_upper_bound: float = 0.3,
    height_offset_lower_bound: float = -0.1,
    height_offset_upper_bound: float = 0.1,
    random_seed: Optional[int] = None,
    color_sequence: Sequence[Sequence[float]] = MULTICOLOR
) -> Tuple[np.ndarray, List[int]]:
  """Generates a series of stepstones with randomly chosen parameters.

  Each stepstone in this series might have different randomly chosen
  parameters.

  Args:
    pybullet_client: The pybullet client instance.
    start_pos: The starting position (the midpoint of top-left edge) of the
      stepstone.
    num_stones: The number of stepstones in this creation.
    stone_height: The height of the stepstones in meters.
    stone_width: The width of the stepstones in meters.
    stone_length_lower_bound: The lower bound of the stone lengths in meters
      when sampled randomly.
    stone_length_upper_bound: The upper bound of the stone lengths in meters
      when sampled randomly.
    gap_length_lower_bound: The lower bound of the gap distances in meters when
      sampled randomly.
    gap_length_upper_bound: The upper bound of the gap distances in meters when
      sampled randomly.
    height_offset_lower_bound: The lower bound of the stone height offsets in
      meters when sampled randomly.
    height_offset_upper_bound: The upper bound of the stone height offsets in
      meters when sampled randomly.
    random_seed: The random seed of the random number generator.
    color_sequence: A list of (red, green, blue, alpha) colors where each
      element is in [0, 1] and alpha is transparency. The stepstones will cycle
      through these colors.

  Returns:
    The position of the mid point of the right-top edge of the stepstone.
    The pybullet ids of the stepstones.
  """
  end_pos = start_pos
  ids = []
  random_generator = random.Random()
  random_generator.seed(random_seed)
  for _, color in zip(range(num_stones), itertools.cycle(color_sequence)):
    stone_length = random_generator.uniform(stone_length_lower_bound,
                                            stone_length_upper_bound)
    gap_length = random_generator.uniform(gap_length_lower_bound,
                                          gap_length_upper_bound)
    height_offset = random_generator.uniform(height_offset_lower_bound,
                                             height_offset_upper_bound)
    end_pos, step_stone_id = build_one_stepstone(
        pybullet_client=pybullet_client,
        start_pos=end_pos,
        stone_length=stone_length,
        stone_height=stone_height,
        stone_width=stone_width,
        gap_length=gap_length,
        height_offset=height_offset,
        rgba_color=color)
    ids.append(step_stone_id)
  return end_pos, ids


def build_platform_at_origin(
    pybullet_client: bullet_client.BulletClient
) -> Tuple[np.ndarray, int]:
  """Builds a platform for the robot to start standing on.

  Args:
    pybullet_client: The pybullet client instance.

  Returns:
    The position of the mid point of the right-top edge of the platform.
    The pybullet id of the platform.
  """
  end_pos, platform_id = build_one_stepstone(
      pybullet_client=pybullet_client,
      start_pos=(-0.5, 0, 0),
      stone_length=1.0,
      stone_height=0.1,
      stone_width=10.0,
      gap_length=0.0,
      height_offset=0.0,
      rgba_color=GRAY)
  return end_pos, platform_id
