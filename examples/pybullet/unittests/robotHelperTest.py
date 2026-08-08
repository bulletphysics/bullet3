import os
import sys
import unittest

import pybullet

sys.path.append(os.path.join(os.path.dirname(__file__), "..", "gym"))

from pybullet_utils.robot_helper import Joint, RobotHelper


def _add_data_path(physics_client):
  try:
    import pybullet_data
    physics_client.setAdditionalSearchPath(pybullet_data.getDataPath())
  except ImportError:
    data_path = os.path.join(os.path.dirname(__file__), "..", "..", "..", "data")
    physics_client.setAdditionalSearchPath(data_path)


class TestRobotHelper(unittest.TestCase):

  def setUp(self):
    self.cid = pybullet.connect(pybullet.DIRECT)
    _add_data_path(pybullet)
    self.robot = pybullet.loadURDF("r2d2.urdf", [0, 0, 1], physicsClientId=self.cid)
    self.helper = RobotHelper(self.robot, physicsClientId=self.cid)

  def tearDown(self):
    pybullet.disconnect(physicsClientId=self.cid)

  def test_joint_discovery(self):
    self.assertGreater(len(self.helper.joints), 0)
    self.assertIn("head_swivel", self.helper.joints)
    self.assertIn("gripper_extension", self.helper.joints)

  def test_fixed_joints_excluded(self):
    for joint in self.helper.joints.values():
      self.assertNotEqual(joint.type, pybullet.JOINT_FIXED)

  def test_joint_attributes(self):
    joint = self.helper.joints["head_swivel"]
    self.assertIsInstance(joint, Joint)
    self.assertEqual(joint.name, "head_swivel")
    self.assertIsInstance(joint.index, int)
    self.assertEqual(joint.type, pybullet.JOINT_REVOLUTE)

  def test_get_joint_position(self):
    position = self.helper.get_joint_position("head_swivel")
    self.assertAlmostEqual(position, 0.0, places=6)

  def test_get_joint_state(self):
    state = self.helper.get_joint_state("head_swivel")
    self.assertEqual(len(state), 4)

  def test_move_converges(self):
    target = -0.3
    self.helper.joints["head_swivel"].move(target)
    for _ in range(500):
      pybullet.stepSimulation(physicsClientId=self.cid)
    position = self.helper.get_joint_position("head_swivel")
    self.assertAlmostEqual(position, target, places=2)

  def test_set_joint_positions(self):
    targets = {"head_swivel": -0.3, "gripper_extension": -0.2}
    self.helper.set_joint_positions(targets)
    for _ in range(500):
      pybullet.stepSimulation(physicsClientId=self.cid)
    for name, target in targets.items():
      position = self.helper.get_joint_position(name)
      self.assertAlmostEqual(position, target, places=2)

  def test_get_joint_positions(self):
    positions = self.helper.get_joint_positions()
    self.assertEqual(len(positions), len(self.helper.joints))
    self.assertIn("head_swivel", positions)

  def test_reset_joint(self):
    self.helper.joints["head_swivel"].reset(-0.3)
    position = self.helper.get_joint_position("head_swivel")
    self.assertAlmostEqual(position, -0.3, places=6)

  def test_get_velocity_and_torque(self):
    joint = self.helper.joints["head_swivel"]
    self.assertAlmostEqual(joint.get_velocity(), 0.0, places=6)
    self.assertAlmostEqual(joint.get_torque(), 0.0, places=6)

  def test_unknown_joint_raises(self):
    with self.assertRaises(KeyError):
      self.helper.get_joint_position("no_such_joint")

  def test_multiple_clients(self):
    cid2 = pybullet.connect(pybullet.DIRECT)
    _add_data_path(pybullet)
    robot2 = pybullet.loadURDF("r2d2.urdf", [0, 0, 1], physicsClientId=cid2)
    helper2 = RobotHelper(robot2, physicsClientId=cid2)
    helper2.joints["head_swivel"].reset(-0.3)
    self.assertAlmostEqual(helper2.get_joint_position("head_swivel"), -0.3, places=6)
    self.assertAlmostEqual(self.helper.get_joint_position("head_swivel"), 0.0, places=6)
    pybullet.disconnect(physicsClientId=cid2)


if __name__ == "__main__":
  unittest.main()
