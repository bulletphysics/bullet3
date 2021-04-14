"""Wraps a quadruped walking controller for navigation control."""

from typing import Any
import gin
from gym import spaces
import numpy as np

from pybullet_envs.minitaur.agents.baseline_controller import com_velocity_estimator
from pybullet_envs.minitaur.agents.baseline_controller import locomotion_controller
from pybullet_envs.minitaur.agents.baseline_controller import openloop_gait_generator
from pybullet_envs.minitaur.agents.baseline_controller import raibert_swing_leg_controller
from pybullet_envs.minitaur.agents.baseline_controller import torque_stance_leg_controller

_N_LEGS = 4
_STANCE_DURATION_SECONDS = [
    0.25
] * _N_LEGS  # The stance phase duration for each leg.
_DUTY_FACTOR = [
    0.6
] * _N_LEGS  # Percentage of the leg in the stance phase within the cycle.
_BODY_HEIGHT = 0.45
_INIT_PHASE_FULL_CYCLE = [0, 0.5, 0.5, 0]


def _setup_controller(robot: Any) -> locomotion_controller.LocomotionController:
  """Creates the controller."""
  desired_speed = (0, 0)
  desired_twisting_speed = 0

  gait_generator = openloop_gait_generator.OpenloopGaitGenerator(
      robot,
      stance_duration=_STANCE_DURATION_SECONDS,
      duty_factor=_DUTY_FACTOR,
      initial_leg_phase=_INIT_PHASE_FULL_CYCLE)
  state_estimator = com_velocity_estimator.COMVelocityEstimator(robot)
  sw_controller = raibert_swing_leg_controller.RaibertSwingLegController(
      robot,
      gait_generator,
      state_estimator,
      desired_speed=desired_speed,
      desired_twisting_speed=desired_twisting_speed,
      desired_height=_BODY_HEIGHT,
  )
  st_controller = torque_stance_leg_controller.TorqueStanceLegController(
      robot,
      gait_generator,
      state_estimator,
      desired_speed=desired_speed,
      desired_twisting_speed=desired_twisting_speed,
      desired_body_height=_BODY_HEIGHT,
      body_mass=215 / 9.8,
      body_inertia=(0.07335, 0, 0, 0, 0.25068, 0, 0, 0, 0.25447),
  )

  controller = locomotion_controller.LocomotionController(
      robot=robot,
      gait_generator=gait_generator,
      state_estimator=state_estimator,
      swing_leg_controller=sw_controller,
      stance_leg_controller=st_controller,
      clock=robot.GetTimeSinceReset)
  return controller


def _update_controller_params(
    controller: locomotion_controller.LocomotionController,
    lin_speed: np.ndarray, ang_speed: float):
  """Apply the desired speed and twisting speed."""
  controller.swing_leg_controller.desired_speed = lin_speed
  controller.swing_leg_controller.desired_twisting_speed = ang_speed
  controller.stance_leg_controller.desired_speed = lin_speed
  controller.stance_leg_controller.desired_twisting_speed = ang_speed


@gin.configurable
class WalkingWrapper(object):
  """Wraps a baseline walking controller for Laikago/Vision60."""

  def __init__(self,
               gym_env: Any,
               action_repeat=20,
               speed_bound=(-0.3, 0.3),
               angular_speed_bound=(-0.3, 0.3)):
    """Initialzes the wrapped env.

    Args:
      gym_env: An instance of LocomotionGymEnv.
      action_repeat: Number of control steps to run low level controller with
        the high level inputs per step().
      speed_bound: The min/max of the input speed.
      angular_speed_bound: The min/max of the twisting speed.
    """
    self._gym_env = gym_env
    self._controller = _setup_controller(self._gym_env.robot)
    self._action_repeat = action_repeat

    action_low = np.array(speed_bound)
    action_high = np.array(angular_speed_bound)

    # Overwrite the action space.
    self.action_space = spaces.Box(action_low, action_high)

  def reset(self, *args, **kwargs) -> Any:
    obs = self._gym_env.reset(*args, **kwargs)
    # The robot instance might have been replaced if hard_reset is called. We
    # just recreate the controller.
    self._controller = _setup_controller(self._gym_env.robot)
    self._controller.reset()
    return obs

  def __getattr__(self, attr):
    return getattr(self._gym_env, attr)

  def step(self, action) -> Any:
    """Steps the wrapped env with high level commands.

    Args:
      action: A high level command containing the desired linear and angular
        speed of the robot base. The speed can be adjusted at any time.

    Returns:
      The gym env observation, reward, termination, and additional info.

    """
    lin_speed = np.array((action[0], 0, 0))
    ang_speed = action[1]
    _update_controller_params(self._controller, lin_speed, ang_speed)
    for _ in range(self._action_repeat):
      self._controller.update()
      hybrid_action = self._controller.get_action()
      obs, reward, done, info = self._gym_env.step(hybrid_action)
      if done:
        break
    return obs, reward, done, info
