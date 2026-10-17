import unittest
import pybullet
import time

from utils import allclose, dot


class TestPybulletMethods(unittest.TestCase):

  def test_import(self):
    import pybullet as p
    self.assertGreater(p.getAPIVersion(), 201700000)

  def test_connect_direct(self):
    import pybullet as p
    cid = p.connect(p.DIRECT)
    self.assertEqual(cid, 0)
    p.disconnect()

  def test_loadurdf(self):
    import pybullet as p
    p.connect(p.DIRECT)
    ob = p.loadURDF("r2d2.urdf")
    self.assertEqual(ob, 0)
    p.disconnect()

  def test_rolling_friction(self):
    import pybullet as p
    p.connect(p.DIRECT)
    p.loadURDF("plane.urdf")
    sphere = p.loadURDF("sphere2.urdf", [0, 0, 1])
    p.resetBaseVelocity(sphere, linearVelocity=[1, 0, 0])
    p.changeDynamics(sphere, -1, linearDamping=0, angularDamping=0)
    #p.changeDynamics(sphere,-1,rollingFriction=0)
    p.setGravity(0, 0, -10)
    for i in range(1000):
      p.stepSimulation()
    vel = p.getBaseVelocity(sphere)
    self.assertLess(vel[0][0], 1e-10)
    self.assertLess(vel[0][1], 1e-10)
    self.assertLess(vel[0][2], 1e-10)
    self.assertLess(vel[1][0], 1e-10)
    self.assertLess(vel[1][1], 1e-10)
    self.assertLess(vel[1][2], 1e-10)
    p.disconnect()

  def test_inverse_dynamics_cache_invalidation(self):
    import pybullet as p

    p.connect(p.DIRECT)
    p.setGravity(0, 0, -10)

    def create_body(link_mass):
      return p.createMultiBody(
          baseMass=0.0,
          baseCollisionShapeIndex=-1,
          basePosition=[0, 0, 0],
          linkMasses=[link_mass],
          linkCollisionShapeIndices=[-1],
          linkVisualShapeIndices=[-1],
          linkPositions=[[0, 0, 1]],
          linkOrientations=[[0, 0, 0, 1]],
          linkInertialFramePositions=[[1, 0, 0]],
          linkInertialFrameOrientations=[[0, 0, 0, 1]],
          linkParentIndices=[0],
          linkJointTypes=[p.JOINT_REVOLUTE],
          linkJointAxis=[[0, 1, 0]],
      )

    # Baseline 10 kg oracle
    body_oracle = create_body(10.0)
    expected_inv = p.calculateInverseDynamics(body_oracle, [0.0], [0.0], [1.0])
    expected_mass_mat = p.calculateMassMatrix(body_oracle, [0.0])
    p.removeBody(body_oracle)

    self.assertAlmostEqual(expected_inv[0], -90.0, places=4)
    self.assertAlmostEqual(expected_mass_mat[0][0], 10.0, places=4)

    # 1. Test changeDynamics invalidates inverse dynamics & mass matrix cache
    body_mutate = create_body(1.0)
    inv_before = p.calculateInverseDynamics(body_mutate, [0.0], [0.0], [1.0])
    mass_before = p.calculateMassMatrix(body_mutate, [0.0])
    self.assertAlmostEqual(inv_before[0], -9.0, places=4)
    self.assertAlmostEqual(mass_before[0][0], 1.0, places=4)

    p.changeDynamics(body_mutate, 0, mass=10.0)
    inv_after = p.calculateInverseDynamics(body_mutate, [0.0], [0.0], [1.0])
    mass_after = p.calculateMassMatrix(body_mutate, [0.0])
    self.assertAlmostEqual(inv_after[0], expected_inv[0], places=4)
    self.assertAlmostEqual(mass_after[0][0], expected_mass_mat[0][0], places=4)
    p.removeBody(body_mutate)

    # 2. Test removeBody purges cached tree so recycled body does not use stale cache
    old_body = create_body(1.0)
    p.calculateInverseDynamics(old_body, [0.0], [0.0], [1.0])
    p.calculateMassMatrix(old_body, [0.0])
    p.removeBody(old_body)

    replacement_body = create_body(10.0)
    inv_replacement = p.calculateInverseDynamics(replacement_body, [0.0], [0.0], [1.0])
    mass_replacement = p.calculateMassMatrix(replacement_body, [0.0])
    self.assertAlmostEqual(inv_replacement[0], expected_inv[0], places=4)
    self.assertAlmostEqual(mass_replacement[0][0], expected_mass_mat[0][0], places=4)
    p.removeBody(replacement_body)

    p.disconnect()


class TestPybulletJacobian(unittest.TestCase):

  def getMotorJointStates(self, robot):
    import pybullet as p
    joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
    joint_infos = [p.getJointInfo(robot, i) for i in range(p.getNumJoints(robot))]
    joint_states = [j for j, i in zip(joint_states, joint_infos) if i[3] > -1]
    joint_positions = [state[0] for state in joint_states]
    joint_velocities = [state[1] for state in joint_states]
    joint_torques = [state[3] for state in joint_states]
    return joint_positions, joint_velocities, joint_torques

  def setJointPosition(self, robot, position, kp=1.0, kv=0.3):
    import pybullet as p
    num_joints = p.getNumJoints(robot)
    zero_vec = [0.0] * num_joints
    if len(position) == num_joints:
      p.setJointMotorControlArray(robot,
                                  range(num_joints),
                                  p.POSITION_CONTROL,
                                  targetPositions=position,
                                  targetVelocities=zero_vec,
                                  positionGains=[kp] * num_joints,
                                  velocityGains=[kv] * num_joints)

  def testJacobian(self):
    import pybullet as p

    clid = p.connect(p.SHARED_MEMORY)
    if (clid < 0):
      p.connect(p.DIRECT)

    time_step = 0.001
    gravity_constant = -9.81

    urdfs = [
        "TwoJointRobot_w_fixedJoints.urdf", "TwoJointRobot_w_fixedJoints.urdf",
        "kuka_iiwa/model.urdf", "kuka_lwr/kuka.urdf"
    ]
    for urdf in urdfs:
      p.resetSimulation()
      p.setTimeStep(time_step)
      p.setGravity(0.0, 0.0, gravity_constant)

      robotId = p.loadURDF(urdf, useFixedBase=True)
      p.resetBasePositionAndOrientation(robotId, [0, 0, 0], [0, 0, 0, 1])
      numJoints = p.getNumJoints(robotId)
      endEffectorIndex = numJoints - 1

      # Set a joint target for the position control and step the sim.
      self.setJointPosition(robotId, [0.1 * (i % 3) for i in range(numJoints)])
      p.stepSimulation()

      # Get the joint and link state directly from Bullet.
      mpos, mvel, mtorq = self.getMotorJointStates(robotId)

      result = p.getLinkState(robotId,
                              endEffectorIndex,
                              computeLinkVelocity=1,
                              computeForwardKinematics=1)
      link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result
      # Get the Jacobians for the CoM of the end-effector link.
      # Note that in this example com_rot = identity, and we would need to use com_rot.T * com_trn.
      # The localPosition is always defined in terms of the link frame coordinates.

      zero_vec = [0.0] * len(mpos)
      jac_t, jac_r = p.calculateJacobian(robotId, endEffectorIndex, com_trn, mpos, zero_vec,
                                         zero_vec)

      assert (allclose(dot(jac_t, mvel), link_vt))
      assert (allclose(dot(jac_r, mvel), link_vr))
    p.disconnect()


if __name__ == '__main__':
  unittest.main()
