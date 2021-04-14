
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function


def get_robot_base_position(robot):
  """Gets the base position of robot."""
  # TODO(b/151975607): Clean this after robot interface migration.
  if hasattr(robot, "GetBasePosition"):
    return robot.GetBasePosition()
  else:
    return robot.base_position


def get_robot_base_orientation(robot):
  """Gets the base orientation of robot."""
  # TODO(b/151975607): Clean this after robot interface migration.
  if hasattr(robot, "GetBaseOrientation"):
    return robot.GetBaseOrientation()
  else:
    return robot.base_orientation_quaternion