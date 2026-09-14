"""Load and exercise the OneRobotics A1 robot assets."""

import argparse
import math
import time

import pybullet as p
import pybullet_data


MODEL_PATHS = {
    "right": "onerobotics_a1/right_arm/model.urdf",
    "left": "onerobotics_a1/left_arm/model.urdf",
    "bimanual": "onerobotics_a1/bimanual_stand/model.urdf",
}
EXPECTED_MOVABLE_JOINTS = {"right": 7, "left": 7, "bimanual": 14}


def safe_target(lower_limit, upper_limit):
  """Return a small target position inside a joint's limits."""
  margin = min(0.05, (upper_limit - lower_limit) * 0.1)
  return min(max(0.15, lower_limit + margin), upper_limit - margin)


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--model", choices=MODEL_PATHS, default="right")
  parser.add_argument(
      "--direct", action="store_true", help="run without a GUI and exit after the motion check"
  )
  args = parser.parse_args()

  client = p.connect(p.DIRECT if args.direct else p.GUI)
  if client < 0:
    raise RuntimeError("Unable to connect to PyBullet")

  try:
    p.resetSimulation(physicsClientId=client)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
    p.setGravity(0, 0, -9.81, physicsClientId=client)
    p.setTimeStep(1.0 / 200.0, physicsClientId=client)
    p.loadURDF("plane.urdf", physicsClientId=client)
    robot = p.loadURDF(
        MODEL_PATHS[args.model],
        useFixedBase=True,
        flags=p.URDF_USE_INERTIA_FROM_FILE,
        physicsClientId=client,
    )

    movable_joints = []
    initial_positions = []
    targets = []
    for joint in range(p.getNumJoints(robot, physicsClientId=client)):
      info = p.getJointInfo(robot, joint, physicsClientId=client)
      if info[2] == p.JOINT_FIXED:
        continue
      movable_joints.append(joint)
      initial_positions.append(p.getJointState(robot, joint, physicsClientId=client)[0])
      targets.append(safe_target(info[8], info[9]))

    expected = EXPECTED_MOVABLE_JOINTS[args.model]
    if len(movable_joints) != expected:
      raise RuntimeError(
          "Expected {} movable joints, found {}".format(expected, len(movable_joints)))

    for order, (joint, target) in enumerate(zip(movable_joints, targets)):
      force = 26.859 if order % 7 < 3 else 5.975
      p.setJointMotorControl2(
          robot,
          joint,
          p.POSITION_CONTROL,
          targetPosition=target,
          force=force,
          positionGain=0.3,
          velocityGain=1.0,
          physicsClientId=client,
      )

    for _ in range(480):
      p.stepSimulation(physicsClientId=client)

    final_positions = [
        p.getJointState(robot, joint, physicsClientId=client)[0] for joint in movable_joints
    ]
    if not all(math.isfinite(position) for position in final_positions):
      raise RuntimeError("The simulation produced a non-finite joint position")
    if not any(
        abs(after - before) > 1e-5
        for before, after in zip(initial_positions, final_positions)
    ):
      raise RuntimeError("No movable joint responded to the position command")

    print("Loaded OneRobotics A1 {}; movable joints: {}".format(args.model,
                                                                 len(movable_joints)))

    if not args.direct:
      while p.isConnected(client):
        p.stepSimulation(physicsClientId=client)
        time.sleep(1.0 / 200.0)
  finally:
    if p.isConnected(client):
      p.disconnect(physicsClientId=client)


if __name__ == "__main__":
  main()
