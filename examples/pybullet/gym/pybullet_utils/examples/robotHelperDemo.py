"""Demo of pybullet_utils.robot_helper driving an R2D2 robot by joint name."""
import math
import time

import pybullet

from pybullet_utils.bullet_client import BulletClient
from pybullet_utils.robot_helper import RobotHelper


def main():
  p = BulletClient(pybullet.GUI)
  try:
    import pybullet_data
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
  except ImportError:
    pass

  p.setGravity(0, 0, -10)
  p.loadURDF("plane.urdf")
  robot_id = p.loadURDF("r2d2.urdf", [0, 0, 1])

  robot = RobotHelper(robot_id, physicsClientId=p)
  print("Controllable joints:", sorted(robot.joints.keys()))

  start_time = time.time()
  next_print = 0.0
  while p.isConnected():
    t = time.time() - start_time
    robot.joints["head_swivel"].move(-0.19 + 0.18 * math.sin(t))
    robot.joints["gripper_extension"].move(-0.19 + 0.18 * math.sin(2 * t))
    if t >= next_print:
      print("head_swivel position: %.3f" % robot.get_joint_position("head_swivel"))
      next_print = t + 0.5
    p.stepSimulation()
    time.sleep(1. / 240.)

  p.disconnect()


if __name__ == "__main__":
  main()
