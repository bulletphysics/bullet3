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
