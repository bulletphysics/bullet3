# Lint as: python3
"""A torque based stance controller framework."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import functools
import logging
from typing import Any, Sequence, Tuple

import gin
import numpy as np

from pybullet_envs.minitaur.agents.baseline_controller import gait_generator as gait_generator_lib
from pybullet_envs.minitaur.agents.baseline_controller import leg_controller
#from pybullet_envs.minitaur.agents.baseline_controller.convex_mpc.python import convex_mpc
#from google3.util.task.python import error

try:
  import mpc_osqp as convex_mpc  # pytype: disable=import-error
except:  #pylint: disable=W0702
  print("You need to install motion_imitation")
  print("or use pip3 install motion_imitation --user")
  print("see also https://github.com/google-research/motion_imitation")
  import sys
  sys.exit()
  

_FORCE_DIMENSION = 3
# The QP weights in the convex MPC formulation. See the MIT paper for details:
#   https://ieeexplore.ieee.org/document/8594448/
# Intuitively, this is the weights of each state dimension when tracking a
# desired CoM trajectory. The full CoM state is represented by
# (roll_pitch_yaw, position, angular_velocity, velocity, gravity_place_holder).
_MPC_WEIGHTS = (5, 5, 0.2, 0, 0, 10, 0.5, 0.5, 0.2, 0.2, 0.2, 0.1, 0)
_PLANNING_HORIZON_STEPS = 10
_PLANNING_TIMESTEP = 0.025
#_MPC_CONSTRUCTOR = functools.partial(
#    convex_mpc.ConvexMpc, qp_solver_name=convex_mpc.QPSolverName.QPOASES)


@gin.configurable
class TorqueStanceLegController(leg_controller.LegController):
  """A torque based stance leg controller framework.

  Takes in high level parameters like walking speed and turning speed, and
  generates necessary the torques for stance legs.
  """

  def __init__(
      self,
      robot: Any,
      gait_generator: Any,
      state_estimator: Any,
      desired_speed: Tuple[float] = (0, 0),
      desired_twisting_speed: float = 0,
      desired_roll_pitch: Tuple[float] = (0, 0),
      desired_body_height: float = 0.45,
      body_mass: float = 220 / 9.8,
      body_inertia: Tuple[float] = (0.183375, 0, 0, 0, 0.6267, 0, 0, 0,
                                    0.636175),
      num_legs: int = 4,
      friction_coeffs: Sequence[float] = (0.5, 0.5, 0.5, 0.5),
      qp_weights: Sequence[float] = _MPC_WEIGHTS,
      planning_horizon: int = _PLANNING_HORIZON_STEPS,
      planning_timestep: int = _PLANNING_TIMESTEP,
  ):
    """Initializes the class.

    Tracks the desired position/velocity of the robot by computing proper joint
    torques using MPC module.

    Args:
      robot: A robot instance.
      gait_generator: Used to query the locomotion phase and leg states.
      state_estimator: Estimate the robot states (e.g. CoM velocity).
      desired_speed: desired CoM speed in x-y plane.
      desired_twisting_speed: desired CoM rotating speed in z direction.
      desired_roll_pitch: desired CoM roll and pitch.
      desired_body_height: The standing height of the robot.
      body_mass: The total mass of the robot.
      body_inertia: The inertia matrix in the body principle frame. We assume
        the body principle coordinate frame has x-forward and z-up.
      num_legs: The number of legs used for force planning.
      friction_coeffs: The friction coeffs on the contact surfaces.
      qp_weights: The weights used in solving the QP problem.
      planning_horizon: Number of steps to roll-out in the QP formulation.
      planning_timestep: Timestep between each step in the QP formulation.
    """

    self._robot = robot
    self._gait_generator = gait_generator
    self._state_estimator = state_estimator
    self._desired_speed = desired_speed
    self._desired_twisting_speed = desired_twisting_speed
    self._desired_roll_pitch = desired_roll_pitch
    self._desired_body_height = desired_body_height
    self._body_mass = body_mass
    self._num_legs = num_legs
    self._friction_coeffs = np.array(friction_coeffs)
    self._qp_solver_fail = False
    self._com_estimate_leg_indices = None
    qp_solver = convex_mpc.QPOASES #convex_mpc.OSQP #
    
    body_inertia_list = list(body_inertia)
    weights_list = list(qp_weights)

    self._mpc = convex_mpc.ConvexMpc(
        body_mass,
        body_inertia_list,
        self._num_legs,
        planning_horizon,
        planning_timestep,
        weights_list,
        1e-6,
        qp_solver
    )
    

  def reset(self, current_time):
    del current_time
    self._qp_solver_fail = False
    self._com_estimate_leg_indices = None

  def update(self, current_time):
    del current_time

  def get_action(self):
    """Computes the torque for stance legs."""
    desired_com_position = np.array((0., 0., self._desired_body_height),
                                    dtype=np.float64)
    desired_com_velocity = np.array(
        (self.desired_speed[0], self.desired_speed[1], 0.), dtype=np.float64)
    desired_com_roll_pitch_yaw = np.array(
        (self.desired_roll_pitch[0], self.desired_roll_pitch[1], 0.),
        dtype=np.float64)
    desired_com_angular_velocity = np.array(
        (0., 0., self.desired_twisting_speed), dtype=np.float64)
    foot_contact_state = np.array(
        [(leg_state == gait_generator_lib.LegState.STANCE or
          leg_state == gait_generator_lib.LegState.EARLY_CONTACT)
         for leg_state in self._gait_generator.desired_leg_state],
        dtype=np.int32)

    # We use the body yaw aligned world frame for MPC computation.
    com_roll_pitch_yaw = np.array(
        self._robot.base_roll_pitch_yaw, dtype=np.float64)
    com_roll_pitch_yaw[2] = 0
    #try:
    estimated_com_position = np.array(())
    if hasattr(self._state_estimator, "estimated_com_height"):
        estimated_com_position = np.array(
            (0, 0, self._state_estimator.estimated_com_height))
    try:
      predicted_contact_forces = self._mpc.compute_contact_forces(
        estimated_com_position,  #com_position
        np.asarray(self._state_estimator.com_velocity_body_yaw_aligned_frame,
                   dtype=np.float64),  #com_velocity
        np.array(com_roll_pitch_yaw, dtype=np.float64),  #com_roll_pitch_yaw
        # Angular velocity in the yaw aligned world frame is actually different
        # from rpy rate. We use it here as a simple approximation.
        np.asarray(self._robot.base_roll_pitch_yaw_rate,
                   dtype=np.float64),  #com_angular_velocity
        foot_contact_state,  #foot_contact_states
        np.array(self._robot.foot_positions(
              position_in_world_frame=False).flatten(),
                 dtype=np.float64),  #foot_positions_base_frame
        self._friction_coeffs,  #foot_friction_coeffs
        desired_com_position,  #desired_com_position
        desired_com_velocity,  #desired_com_velocity
        desired_com_roll_pitch_yaw,  #desired_com_roll_pitch_yaw
        desired_com_angular_velocity  #desired_com_angular_velocity
    )
    except:# error.StatusNotOk as e:
      logging.error("Error in Torque Stance Leg")#e.message)
      self._qp_solver_fail = True
      predicted_contact_forces = np.zeros(self._num_legs * _FORCE_DIMENSION)

    contact_forces = {}
    for i in range(self._num_legs):
      contact_forces[i] = np.array(
          predicted_contact_forces[i * _FORCE_DIMENSION:(i + 1) *
                                   _FORCE_DIMENSION])

    _, kds = self._robot.motor_model.get_motor_gains()
    action = {}
    for leg_id, force in contact_forces.items():
      motor_torques = self._robot.map_contact_force_to_joint_torques(
          leg_id, force)
      for joint_id, torque in motor_torques.items():
        action[joint_id] = (0, 0, 0, kds[joint_id], torque)
    return action

  @property
  def qp_solver_fail(self):
    return self._qp_solver_fail

  @property
  def desired_speed(self):
    return self._desired_speed

  @desired_speed.setter
  def desired_speed(self, speed):
    self._desired_speed = speed

  @property
  def desired_twisting_speed(self):
    return self._desired_twisting_speed

  @desired_twisting_speed.setter
  def desired_twisting_speed(self, twisting_speed):
    self._desired_twisting_speed = twisting_speed

  @property
  def desired_roll_pitch(self):
    return self._desired_roll_pitch

  @desired_roll_pitch.setter
  def desired_roll_pitch(self, roll_pitch):
    self._desired_roll_pitch = roll_pitch

  @property
  def desired_body_height(self):
    return self._desired_body_height

  @desired_body_height.setter
  def desired_body_height(self, body_height):
    self._desired_body_height = body_height
