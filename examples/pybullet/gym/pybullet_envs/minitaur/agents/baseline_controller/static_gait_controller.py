# Lint as: python3
"""A static gait controller for a quadruped robot. Experimental code."""

import gin
import numpy as np
from pybullet_envs.minitaur.agents.baseline_controller import foot_stepper

toe_pos_local_ref = np.array([[0.1478, -0.11459, -0.45576],
                              [0.1478, 0.11688, -0.45576],
                              [-0.2895, -0.11459, -0.45576],
                              [-0.2895, 0.11688, -0.45576]])


@gin.configurable
class StaticGaitController(object):
  """A static gait controller for a quadruped robot."""

  def __init__(self, robot):
    self._robot = robot
    self._toe_ids = tuple(robot.urdf_loader.get_end_effector_id_dict().values())
    self._wait_count = 0
    self._stepper = foot_stepper.FootStepper(self._robot.pybullet_client,
                                             self._toe_ids, toe_pos_local_ref)

  def act(self, observation):
    """Computes actions based on observations."""
    del observation
    p = self._robot.pybullet_client
    quadruped = self._robot.robot_id
    step_input = foot_stepper.StepInput()
    ls = p.getLinkStates(
        quadruped, self._toe_ids, computeForwardKinematics=True)
    toe_pos_world = np.array([ls[0][0], ls[1][0], ls[2][0], ls[3][0]])
    base_com_pos, base_com_orn = p.getBasePositionAndOrientation(quadruped)
    new_pos_world = np.array([0, 0, 0])

    if self._stepper.is_com_stable() and not self._stepper.move_swing_foot:
      self._wait_count += 1
      if self._wait_count == 20:
        self._stepper.next_foot()
      if self._wait_count > 50:
        self._wait_count = 0
        step_dist = 0.15
        print("time {}, make a step of {}".format(
            self._robot.GetTimeSinceReset(), step_dist))
        new_pos_local = self._stepper.get_reference_pos_swing_foot()
        new_pos_local[0] += step_dist
        new_pos_world, _ = p.multiplyTransforms(base_com_pos, base_com_orn,
                                                new_pos_local, [0, 0, 0, 1])
        self._stepper.swing_foot()

    step_input.new_pos_world = new_pos_world
    step_input.base_com_pos = base_com_pos
    step_input.base_com_orn = base_com_orn
    step_input.toe_pos_world = toe_pos_world
    step_input.dt = 1.0 / 250
    step_output = self._stepper.update(step_input)

    # Finds joint poses to achieve toePosWorld
    desired_joint_angles = self._robot.motor_angles_from_foot_positions(
        foot_positions=step_output.new_toe_pos_world,
        position_in_world_frame=True)[1]
    return desired_joint_angles
