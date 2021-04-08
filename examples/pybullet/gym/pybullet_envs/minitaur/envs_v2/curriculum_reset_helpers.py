"""Implements dynamic locomotion gym env that changes based on iteration."""
import gin

import pybullet


# TODO(b/142642890): Make this reset to the initial world state first.
@gin.configurable
def gap_task_curriculum_update(env,
                               num_iter,
                               distance_to_gap_or_hurdle=1.5,
                               initial_gap_length=0.1,
                               max_iterations=500,
                               gap_delta=0.0008):
  """Linearly increase the gap width wrt the iteration number.

  This is specific to BuildSingleGapWorld.

  Args:
    env: An instance of a LocomotionGymEnv.
    num_iter: The training iteration we are on.
    distance_to_gap_or_hurdle: The distance to the gap.
    initial_gap_length: The starting gap length.
    max_iterations: The number of iterations up to which we will modify the
      environment.
    gap_delta: The amount to increase the gap width by for each increase of 1 in
      the iteration.
  """

  gap_length = initial_gap_length + gap_delta * min(max_iterations, num_iter)
  env.task.reset(
      env,
      distance_to_gap_or_hurdle=distance_to_gap_or_hurdle,
      gap_or_hurdle_width=gap_length)


@gin.configurable
def gap_world_curriculum_update(env,
                                num_iter,
                                initial_second_block_x=8.15,
                                max_iterations=500,
                                gap_delta=0.0008):
  """Update the world, linearly increasing gap width wrt iteration number.

  This is specific to SingleGapScene.

  Args:
    env: An instance of a LocomotionGymEnv.
    num_iter: The training iteration we are on.
    initial_second_block_x: The initial x position of the second block.
    max_iterations: The number of iterations up to which we will modify the
      environment.
    gap_delta: The amount to increase the gap width by for each increase of 1 in
      the iteration.
  """

  ground = env.scene.ground_ids
  pos = pybullet.getBasePositionAndOrientation(ground[-1])[0]
  # Linearly increase the gap width to 0.5m by the last iteration.
  next_x = initial_second_block_x + gap_delta * min(max_iterations, num_iter)
  pybullet.resetBasePositionAndOrientation(ground[-1], (next_x, pos[1], pos[2]),
                                           [0, 0, 0, 1])
