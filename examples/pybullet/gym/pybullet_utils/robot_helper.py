"""Beginner-friendly helpers for controlling robots in pybullet.

Example usage:

  import pybullet
  from pybullet_utils.robot_helper import RobotHelper

  pybullet.connect(pybullet.GUI)
  robot_id = pybullet.loadURDF("r2d2.urdf", [0, 0, 1])

  robot = RobotHelper(robot_id)
  robot.joints["head_swivel"].move(1.0)
  position = robot.get_joint_position("head_swivel")
"""
from __future__ import absolute_import
from __future__ import division

import pybullet


class Joint(object):
  """A single controllable joint of a robot.

  Attributes:
    name: Name of the joint as defined in the URDF.
    index: Joint index used by the pybullet API.
    type: Joint type, e.g. `pybullet.JOINT_REVOLUTE` or `pybullet.JOINT_PRISMATIC`.
    lower_limit: Lower joint limit in radians or meters.
    upper_limit: Upper joint limit in radians or meters.
    max_force: Maximum force the joint can apply.
    max_velocity: Maximum joint velocity.
  """

  def __init__(self, robot_id, index, name, joint_type, lower_limit, upper_limit, max_force,
               max_velocity, pb, call_kwargs):
    """Creates a Joint.

    Args:
      robot_id: Body unique id returned by `pybullet.loadURDF`.
      index: Joint index used by the pybullet API.
      name: Name of the joint as defined in the URDF.
      joint_type: Joint type, e.g. `pybullet.JOINT_REVOLUTE`.
      lower_limit: Lower joint limit in radians or meters.
      upper_limit: Upper joint limit in radians or meters.
      max_force: Maximum force the joint can apply.
      max_velocity: Maximum joint velocity.
      pb: The pybullet module or a `BulletClient` instance.
      call_kwargs: Extra keyword arguments passed to every pybullet call,
        typically `{"physicsClientId": cid}`.
    """
    self._robot_id = robot_id
    self._pb = pb
    self._call_kwargs = call_kwargs
    self.index = index
    self.name = name
    self.type = joint_type
    self.lower_limit = lower_limit
    self.upper_limit = upper_limit
    self.max_force = max_force
    self.max_velocity = max_velocity

  def move(self, position, max_velocity=None, force=None):
    """Moves the joint to a target position using position control.

    Args:
      position: Target position in radians (revolute) or meters (prismatic).
      max_velocity: Optional velocity limit used while approaching the target.
      force: Optional maximum force the joint may apply.
    """
    kwargs = dict(self._call_kwargs)
    if max_velocity is not None:
      kwargs["maxVelocity"] = max_velocity
    if force is not None:
      kwargs["force"] = force
    self._pb.setJointMotorControl2(self._robot_id,
                                   self.index,
                                   self._pb.POSITION_CONTROL,
                                   targetPosition=position,
                                   **kwargs)

  def set_velocity(self, velocity, force=None):
    """Drives the joint at a target velocity using velocity control.

    Args:
      velocity: Target velocity in radians/s (revolute) or meters/s (prismatic).
      force: Optional maximum force the joint may apply.
    """
    kwargs = dict(self._call_kwargs)
    if force is not None:
      kwargs["force"] = force
    self._pb.setJointMotorControl2(self._robot_id,
                                   self.index,
                                   self._pb.VELOCITY_CONTROL,
                                   targetVelocity=velocity,
                                   **kwargs)

  def get_position(self):
    """Returns the current joint position in radians or meters."""
    state = self._pb.getJointState(self._robot_id, self.index, **self._call_kwargs)
    return state[0]

  def get_velocity(self):
    """Returns the current joint velocity in radians/s or meters/s."""
    state = self._pb.getJointState(self._robot_id, self.index, **self._call_kwargs)
    return state[1]

  def get_torque(self):
    """Returns the torque applied by the joint motor in the last simulation step."""
    state = self._pb.getJointState(self._robot_id, self.index, **self._call_kwargs)
    return state[3]

  def reset(self, position, velocity=0.0):
    """Instantly resets the joint to a position without simulating motion.

    Args:
      position: Joint position in radians or meters.
      velocity: Joint velocity in radians/s or meters/s.
    """
    self._pb.resetJointState(self._robot_id, self.index, position, velocity, **self._call_kwargs)

  def __repr__(self):
    return "Joint(name=%r, index=%d, type=%d)" % (self.name, self.index, self.type)


class RobotHelper(object):
  """A lightweight helper that simplifies common robot operations in pybullet.

  Example usage:

    robot = RobotHelper(robot_id)
    robot.joints["head_swivel"].move(1.0)
    position = robot.get_joint_position("head_swivel")
  """

  def __init__(self, robot_id, physicsClientId=-1):
    """Creates a RobotHelper.

    Args:
      robot_id: Body unique id returned by `pybullet.loadURDF`.
      physicsClientId: Physics client id returned by `pybullet.connect`, or a
        `pybullet_utils.bullet_client.BulletClient` instance. Defaults to -1,
        the default pybullet client.
    """
    self.robot_id = robot_id
    if hasattr(physicsClientId, "getNumJoints"):
      self._pb = physicsClientId
      self._call_kwargs = {}
    else:
      self._pb = pybullet
      self._call_kwargs = {"physicsClientId": physicsClientId}
    self.joints = {}
    self._discover_joints()

  def _discover_joints(self):
    num_joints = self._pb.getNumJoints(self.robot_id, **self._call_kwargs)
    for index in range(num_joints):
      info = self._pb.getJointInfo(self.robot_id, index, **self._call_kwargs)
      joint_type = info[2]
      if joint_type == self._pb.JOINT_FIXED:
        continue
      name = info[1]
      if isinstance(name, bytes):
        name = name.decode("utf-8")
      self.joints[name] = Joint(self.robot_id, index, name, joint_type, info[8], info[9], info[10],
                                info[11], self._pb, self._call_kwargs)

  def get_joint_position(self, name):
    """Returns the current position of a joint.

    Args:
      name: Joint name as defined in the URDF.

    Raises:
      KeyError: If no joint with this name exists.
    """
    return self.joints[name].get_position()

  def get_joint_state(self, name):
    """Returns the full state of a joint.

    Args:
      name: Joint name as defined in the URDF.

    Returns:
      A tuple (position, velocity, reactionForces, appliedJointMotorTorque).

    Raises:
      KeyError: If no joint with this name exists.
    """
    joint = self.joints[name]
    return self._pb.getJointState(self.robot_id, joint.index, **self._call_kwargs)

  def get_joint_positions(self):
    """Returns the current positions of all controllable joints.

    Returns:
      A dict mapping joint name to joint position.
    """
    return {name: joint.get_position() for name, joint in self.joints.items()}

  def set_joint_positions(self, targets):
    """Moves multiple joints to target positions using position control.

    Args:
      targets: A dict mapping joint name to target position.

    Raises:
      KeyError: If a joint name in `targets` does not exist.
    """
    indices = [self.joints[name].index for name in targets]
    positions = [targets[name] for name in targets]
    self._pb.setJointMotorControlArray(self.robot_id,
                                       indices,
                                       self._pb.POSITION_CONTROL,
                                       targetPositions=positions,
                                       **self._call_kwargs)

  def __repr__(self):
    return "RobotHelper(robot_id=%d, joints=%d)" % (self.robot_id, len(self.joints))
