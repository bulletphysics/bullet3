# Lint as: python3
"""A state machine that steps each foot for a static gait. Experimental code."""

import copy
import math

import numpy as np


class StepInput(object):

  def __init__(self):
    self.base_com_pos = np.array([0, 0, 0])
    self.base_com_orn = np.array([0, 0, 0, 1])
    self.toe_pos_world = np.array([0, 0, 0] * 4)
    self.new_pos_world = np.array([0, 0, 0])


class StepOutput(object):

  def __init__(self, new_toe_pos_world):
    self.new_toe_pos_world = new_toe_pos_world


class FootStepper(object):
  """This class computes desired foot placement for a quadruped robot."""

  def __init__(self, bullet_client, toe_ids, toe_pos_local_ref):
    self.bullet_client = bullet_client
    self.state_time = 0.
    self.toe_ids = toe_ids
    self.toe_pos_local_ref = toe_pos_local_ref
    self.sphere_uid = self.bullet_client.loadURDF(
        "sphere_small.urdf", [0, 0, 0], useFixedBase=True)
    self.is_far = True
    self.max_shift = 0.0008
    self.far_bound = 0.005
    self.close_bound = 0.03

    self.move_swing_foot = False
    self.amp = 0.2
    alpha = 1

    # Loads/draws spheres for debugging purpose. The spheres visualize the
    # target COM, the current COM and the target foothold location.
    self.sphere_uid_centroid = self.bullet_client.loadURDF(
        "sphere_small.urdf", [0, 0, 0], useFixedBase=True)
    self.bullet_client.changeVisualShape(
        self.sphere_uid_centroid, -1, rgbaColor=[1, 1, 0, alpha])

    # Disable collision since visualization spheres should not collide with the
    # robot.
    self.bullet_client.setCollisionFilterGroupMask(self.sphere_uid_centroid, -1,
                                                   0, 0)

    self.sphere_uid_com = self.bullet_client.loadURDF(
        "sphere_small.urdf", [0, 0, 0], useFixedBase=True)
    self.bullet_client.changeVisualShape(
        self.sphere_uid_com, -1, rgbaColor=[1, 0, 1, alpha])
    self.bullet_client.setCollisionFilterGroupMask(self.sphere_uid_com, -1, 0,
                                                   0)

    self.bullet_client.setCollisionFilterGroupMask(self.sphere_uid, -1, 0, 0)
    self.feetindices = [1, 3, 0, 2]
    self.swing_foot_index1 = 0
    self.swing_foot_index = self.feetindices[self.swing_foot_index1]
    self.colors = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1], [1, 1, 1, 1]]
    self.support_vertices = [[1, 2, 3], [0, 2, 3], [0, 1, 3], [0, 1, 2]]
    self.local_diff_y_threshold = 0.05
    self.local_diff_y = 100
    self.is_far = True
    self.get_reference_pos_swing_foot()

  def next_foot(self):
    self.swing_foot_index1 = (self.swing_foot_index1 + 1) % 4
    self.swing_foot_index = self.feetindices[self.swing_foot_index1]

  def swing_foot(self):
    self.move_swing_foot = True

  def get_reference_pos_swing_foot(self):
    self.new_pos_local = np.array(
        self.toe_pos_local_ref[self.swing_foot_index])
    return self.new_pos_local

  def set_reference_pos_swing_foot(self, new_pos_local):
    self.new_pos_local = new_pos_local

  def is_com_stable(self):
    ld2 = self.local_diff_y * self.local_diff_y
    yaw_ok = ld2 < (self.local_diff_y_threshold * self.local_diff_y_threshold)
    com_ok = not self.is_far
    return com_ok and yaw_ok

  def update(self, step_input):
    """Updates the state machine and toe movements per state."""
    base_com_pos = step_input.base_com_pos
    base_com_orn = step_input.base_com_orn
    base_com_pos_inv, base_com_orn_inv = self.bullet_client.invertTransform(
        base_com_pos, base_com_orn)

    dt = step_input.dt
    self.bullet_client.resetBasePositionAndOrientation(self.sphere_uid,
                                                       step_input.new_pos_world,
                                                       [0, 0, 0, 1])
    self.bullet_client.changeVisualShape(
        self.sphere_uid, -1, rgbaColor=self.colors[self.swing_foot_index])

    all_toes_pos_locals = []
    for toe_pos_world in step_input.toe_pos_world:
      toe_pos_local, _ = self.bullet_client.multiplyTransforms(
          base_com_pos_inv, base_com_orn_inv, toe_pos_world, [0, 0, 0, 1])
      all_toes_pos_locals.append(toe_pos_local)
    all_toes_pos_locals = np.array(all_toes_pos_locals)
    centroid_world = np.zeros(3)
    for v in self.support_vertices[self.swing_foot_index]:
      vtx_pos_world = step_input.toe_pos_world[v]
      centroid_world += vtx_pos_world
    centroid_world /= 3.

    sphere_z_offset = 0.05
    self.diff_world = base_com_pos - centroid_world
    self.diff_world[2] = 0.
    self.bullet_client.resetBasePositionAndOrientation(self.sphere_uid_centroid,
                                                       centroid_world,
                                                       [0, 0, 0, 1])
    self.bullet_client.resetBasePositionAndOrientation(
        self.sphere_uid_com,
        [base_com_pos[0], base_com_pos[1], sphere_z_offset], [0, 0, 0, 1])

    l = np.linalg.norm(self.diff_world)
    if self.is_far:
      bound = self.far_bound
    else:
      bound = self.close_bound

    if l > bound:
      self.diff_world *= self.max_shift * 0.5 / l
      if not self.is_far:
        self.is_far = True
    else:
      if self.is_far:
        self.is_far = False

    if not self.is_far:
      self.diff_world = np.zeros(3)
    for i in range(len(self.toe_pos_local_ref)):
      toe = self.toe_pos_local_ref[i]
      toe = [
          toe[0] + self.diff_world[0], toe[1] + self.diff_world[1],
          toe[2] + self.diff_world[2]
      ]
      self.toe_pos_local_ref[i] = toe

    self.local_diff_y = self.toe_pos_local_ref[0][
        1] + self.toe_pos_local_ref[1][1] - self.toe_pos_local_ref[
            2][1] - self.toe_pos_local_ref[3][1]

    self.yaw = 0
    if self.local_diff_y < -self.local_diff_y_threshold:
      self.yaw = 0.001
    if self.local_diff_y > self.local_diff_y_threshold:
      self.yaw = -0.001

    yaw_trans = self.bullet_client.getQuaternionFromEuler([0, 0, self.yaw])

    if not self.is_far:
      for i in range(len(self.toe_pos_local_ref)):
        toe = self.toe_pos_local_ref[i]
        toe, _ = self.bullet_client.multiplyTransforms([0, 0, 0], yaw_trans,
                                                       toe, [0, 0, 0, 1])
        self.toe_pos_local_ref[i] = toe

    new_toe_pos_world = []

    # Moves the swing foot to the target location.
    if self.move_swing_foot:
      if self.state_time <= 1:
        self.state_time += 4 * dt
    if self.state_time >= 1:
      self.move_swing_foot = False
      self.state_time = 0
      self.toe_pos_local_ref[self.swing_foot_index] = self.new_pos_local
    toe_pos_local_ref_copy = copy.deepcopy(self.toe_pos_local_ref)
    old_pos = self.toe_pos_local_ref[self.swing_foot_index]
    new_pos = [
        old_pos[0] * (1 - self.state_time) + self.new_pos_local[0] *
        (self.state_time), old_pos[1] * (1 - self.state_time) +
        self.new_pos_local[1] * (self.state_time),
        old_pos[2] * (1 - self.state_time) + self.new_pos_local[2] *
        (self.state_time) + self.amp * math.sin(self.state_time * math.pi)
    ]
    toe_pos_local_ref_copy[self.swing_foot_index] = new_pos
    for toe_pos_local in toe_pos_local_ref_copy:
      new_toe_pos_world.append(self.bullet_client.multiplyTransforms(
          base_com_pos, base_com_orn, toe_pos_local, [0, 0, 0, 1])[0])

    step_output = StepOutput(new_toe_pos_world)
    return step_output
