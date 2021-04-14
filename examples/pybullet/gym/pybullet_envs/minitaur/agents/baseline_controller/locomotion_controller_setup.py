"""The common setups for MPC based locoomtion controller environments."""
import time
import gin
import numpy as np

from pybullet_envs.minitaur.agents.baseline_controller import com_velocity_estimator
from pybullet_envs.minitaur.agents.baseline_controller import imu_based_com_velocity_estimator
from pybullet_envs.minitaur.agents.baseline_controller import locomotion_controller
from pybullet_envs.minitaur.agents.baseline_controller import openloop_gait_generator
from pybullet_envs.minitaur.agents.baseline_controller import raibert_swing_leg_controller
from pybullet_envs.minitaur.agents.baseline_controller import torque_stance_leg_controller
#from pybullet_envs.minitaur.envs.env_randomizers import minitaur_push_randomizer
from pybullet_envs.minitaur.envs.env_randomizers import minitaur_push_randomizer
from pybullet_envs.minitaur.robots import robot_config
import pybullet_data as pd

CONFIG_FILE = (pd.getDataPath()+"/configs_v2/base/laikago_reactive.gin")

_MOTOR_KD = [1.0, 2.0, 2.0] * 4
_BODY_HEIGHT = 0.45
_MAX_TIME_SECONDS = 1000000000
_MOTOR_KD = [1.0, 2.0, 2.0] * 4
# TODO(tingnan): This is for tunining the moments of inertia of the model.
# Once we identified the correct value we can remove this.
_SCALE = 4
_INERTIA = (0.07335 * _SCALE, 0, 0, 0, 0.25068 * _SCALE, 0, 0, 0,
            0.25447 * _SCALE)


def load_sim_config(render=True):
  """Builds the environment for the quadruped robot.

  Args:
    render: Enable/disable rendering.
  """
  gin.clear_config(clear_constants=False)
  config_file = CONFIG_FILE
  gin.parse_config_file(config_file)

  # Sim bindings
  # Overwrite a few parameters.

  action_repeat = 4
  gin.bind_parameter("SimulationParameters.num_action_repeat", action_repeat)
  gin.bind_parameter("laikago_v2.Laikago.action_repeat", action_repeat)

  # Control latency is NOT modeled properly for inverse kinematics and
  # jacobians, as we are directly calling the pybullet API. We will try to fix
  # this by loading a separate pybullet instance, set the pose and joint
  # angles which has latency in them, and then run the jacobian/IK.
  gin.bind_parameter("laikago_v2.Laikago.motor_control_mode",
                     robot_config.MotorControlMode.HYBRID)
  # Bump up a bit the adduction/abduction motor d gain for a better tracking.
  gin.bind_parameter("hybrid_motor_model.HybridMotorModel.kd", _MOTOR_KD)
  gin.bind_parameter("SimulationParameters.enable_rendering", render)
  gin.bind_parameter("env_loader.load.wrapper_classes", [])



def add_random_push_config():
  """Adds a random push randomizers to the config."""
  try:
    current_env_randomizers = gin.query_parameter(
        "locomotion_gym_env.LocomotionGymEnv.env_randomizers")

    current_env_randomizers.append(
        minitaur_push_randomizer.MinitaurPushRandomizer(
            horizontal_force_bound=(500, 900),
            vertical_force_bound=(50, 100),
            visualize_perturbation_force=True))
    gin.bind_parameter("locomotion_gym_env.LocomotionGymEnv.env_randomizers",
                       current_env_randomizers)
  except ValueError:
    # No randoimzers bind so far
    gin.bind_parameter("locomotion_gym_env.LocomotionGymEnv.env_randomizers", [
        minitaur_push_randomizer.MinitaurPushRandomizer(
            horizontal_force_bound=(500, 900),
            vertical_force_bound=(50, 100),
            visualize_perturbation_force=True)
    ])


def select_gait(gait_type="fast_trot"):
  """Selects a gait pattern.

  Args:
    gait_type: which gait to use.

  Returns:
    A tuple of (stance_duration, duty_factor, initial_phase)
  """
  # Each gait is composed of stance_duration, duty_factor, and
  # init_phase_full_cycle.
  if gait_type == "fast_trot":
    return [0.25] * 4, [0.6] * 4, [0, 0.5, 0.5, 0]
  elif gait_type == "slow_trot":
    return [0.4] * 4, [0.65] * 4, [0, 0.5, 0.5, 0]
  elif gait_type == "walk":
    return [0.75] * 4, [0.8] * 4, [0.25, 0.75, 0.5, 0]
  else:
    # Means four leg stand for as long as possible.
    return [_MAX_TIME_SECONDS] * 4, [0.99] * 4, [0, 0, 0, 0]


def setup_controller(robot,
                     gait="fast_trot",
                     run_on_robot=False,
                     use_ground_truth_velocity=False):
  """Demonstrates how to create a locomotion controller.

  Args:
    robot: A robot instance.
    gait: The type of gait to use.
    run_on_robot: Whether this controller is running on the real robot or not.
    use_ground_truth_velocity: Whether to use ground truth velocity or velocity
      estimator.

  Returns:
    A locomotion controller.
  """
  desired_speed = (0, 0)
  desired_twisting_speed = 0

  feet_positions = np.array(robot.foot_positions())
  feet_positions[:, 2] = 0

  # Sim and real robots have different mass and contact detection parameters.
  body_weight, contact_force_threshold = (200, 20) if run_on_robot else (215, 0)

  stance_duration, duty_factor, init_phase = select_gait(gait)
  gait_generator = openloop_gait_generator.OpenloopGaitGenerator(
      robot,
      stance_duration=stance_duration,
      duty_factor=duty_factor,
      initial_leg_phase=init_phase,
      contact_detection_force_threshold=contact_force_threshold,
  )
  state_estimator = (
      imu_based_com_velocity_estimator.IMUBasedCOMVelocityEstimator(
          robot,
          contact_detection_threshold=contact_force_threshold,
      ))

  # Use this in sim to test ground truth velocity estimation.
  if use_ground_truth_velocity:
    state_estimator = com_velocity_estimator.COMVelocityEstimator(robot)

  sw_controller = raibert_swing_leg_controller.RaibertSwingLegController(
      robot,
      gait_generator,
      state_estimator,
      desired_speed=desired_speed,
      desired_twisting_speed=desired_twisting_speed,
      desired_height=_BODY_HEIGHT,
      local_hip_positions=feet_positions,
  )
  st_controller = torque_stance_leg_controller.TorqueStanceLegController(
      robot,
      gait_generator,
      state_estimator,
      desired_speed=desired_speed,
      desired_twisting_speed=desired_twisting_speed,
      desired_body_height=_BODY_HEIGHT,
      body_mass=body_weight / 9.8,
      body_inertia=_INERTIA,
  )

  controller = locomotion_controller.LocomotionController(
      robot=robot,
      gait_generator=gait_generator,
      state_estimator=state_estimator,
      swing_leg_controller=sw_controller,
      stance_leg_controller=st_controller,
      clock=robot.GetTimeSinceReset)
  return controller
