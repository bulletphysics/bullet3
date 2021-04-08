r"""ScenarioSet example for Laikago MPC controller.

blaze run -c opt \
//robotics/reinforcement_learning/minitaur/agents/baseline_controller\
:locomotion_controller_in_scenario_set_example -- --gait=slow_trot \
--add_random_push=True
"""

from absl import app
from absl import flags
import gin
import numpy as np
import scipy.interpolate

from pybullet_envs.minitaur.agents.baseline_controller import locomotion_controller_setup
from pybullet_envs.minitaur.envs_v2 import env_loader

FLAGS = flags.FLAGS

SCENARIO_SET_CONFIG = """
import pybullet_envs.minitaur.envs_v2.scenarios.locomotion_simple_scenario_set

include "google3/robotics/reinforcement_learning/minitaur/envs_v2/scenarios/default_scenario_set.gin"

default_scenario_set/singleton.constructor = @locomotion_simple_scenario_set.LocomotionSimpleScenarioSet


locomotion_simple_scenario_set.LocomotionSimpleScenarioSet.selector = "flat_ground"
locomotion_gym_env.LocomotionGymEnv.task = @scenario_set.task()
locomotion_gym_env.LocomotionGymEnv.scene = @scenario_set.scene()
locomotion_gym_env.LocomotionGymEnv.env_randomizers = [
  @scenario_set.env_randomizer()
]
"""

_MAX_TIME_SECONDS = 30

flags.DEFINE_enum("gait", "fast_trot",
                  ["fast_trot", "slow_trot", "walk", "stand"],
                  "The gait pattern to use")

flags.DEFINE_boolean("add_random_push", False,
                     "whether to add random push to the robot in simulation")


def _start_stop_profile(max_speed=0.5, axis=0, duration=3):
  speed_profile = np.zeros((3, 4))

  speed_profile[1, axis] = max_speed

  return (0, 0.5, duration + 0.5), speed_profile.tolist()


def _random_speed_profile(max_speed=1, axis=0, time_interval=1.0):
  num_pts = 11
  time_points = np.arange(num_pts) * time_interval

  speed_profile = np.zeros((num_pts, 4))
  speed_profile[:, axis] = np.random.uniform(0, max_speed, num_pts)
  speed_profile[-1, :] = 0
  return time_points.tolist(), speed_profile.tolist()


def _body_height_profile(z_range=(0.3, 0.55)):
  del z_range
  # TODO(tingnan): Implement this.


def _generate_linear_angular_speed(t, time_points, speed_points):
  """Creates an example speed profile based on time for demo purpose."""

  speed = scipy.interpolate.interp1d(
      time_points,
      speed_points,
      kind="previous",
      fill_value="extrapolate",
      axis=0)(
          t)

  return speed[0:3], speed[3]


def _update_controller_params(controller, lin_speed, ang_speed):
  controller.swing_leg_controller.desired_speed = lin_speed
  controller.swing_leg_controller.desired_twisting_speed = ang_speed
  controller.stance_leg_controller.desired_speed = lin_speed
  controller.stance_leg_controller.desired_twisting_speed = ang_speed


def _gen_stability_test_start_stop():
  """Generates the speed profile for start/stop tests."""
  axis_to_name = {
      0: "velocity x",
      1: "velocity y",
      3: "angular velocity z",
  }

  axis_to_max_speed = {
      0: 1.0,
      1: 0.5,
      3: 2.5,
  }

  gait_multiplier = {
      "slow_trot": 0.7,
      "walk": 0.3,
      "fast_trot": 1.0,
  }

  for axis in (0, 1, 3):
    yield axis_to_name[axis], _start_stop_profile(
        axis_to_max_speed[axis] * gait_multiplier[FLAGS.gait], axis)


def _gen_stability_test_random():
  """Generates the speed profile for random walking tests."""
  axis_to_name = {
      0: "velocity x",
      1: "velocity y",
      3: "angular velocity z",
  }

  axis_to_max_speed = {
      0: 1.0,
      1: 0.5,
      3: 2.5,
  }

  gait_multiplier = {
      "slow_trot": 0.7,
      "walk": 0.3,
      "fast_trot": 1.0,
  }

  for axis in (0, 1, 3):
    yield axis_to_name[axis], _random_speed_profile(
        axis_to_max_speed[axis] * gait_multiplier[FLAGS.gait], axis)


def _test_stability(max_time=5, render=False, test_generator=None):
  """Tests the stability of the controller using speed profiles."""
  locomotion_controller_setup.load_sim_config(render=render)
  gin.parse_config(SCENARIO_SET_CONFIG)
  if FLAGS.add_random_push:
    locomotion_controller_setup.add_random_push_config()

  env = env_loader.load()
  controller = locomotion_controller_setup.setup_controller(
      env.robot, gait=FLAGS.gait)

  for name, speed_profile in test_generator():
    env.reset()
    controller.reset()
    current_time = 0
    while current_time < max_time:
      current_time = env.get_time_since_reset()
      lin_speed, ang_speed = _generate_linear_angular_speed(
          current_time, speed_profile[0], speed_profile[1])
      _update_controller_params(controller, lin_speed, ang_speed)

      # Needed before every call to get_action().
      controller.update()
      hybrid_action = controller.get_action()

      _, _, done, _ = env.step(hybrid_action)
      if done:
        break
    print(f"Scene name: flat ground. Random push: {FLAGS.add_random_push}. "
          f"Survival time for {name} = {speed_profile[1]} is {current_time}")


def main(argv):
  del argv
  _test_stability(render=True, test_generator=_gen_stability_test_start_stop)
  _test_stability(
      max_time=15, render=True, test_generator=_gen_stability_test_random)


if __name__ == "__main__":
  app.run(main)
