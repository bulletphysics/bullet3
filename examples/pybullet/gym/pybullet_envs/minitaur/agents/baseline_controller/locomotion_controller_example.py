r"""Laikago walking example using the locomotion controller framework.

"""

import gc
import pickle

from absl import app
from absl import flags
import numpy as np
import scipy.interpolate


from pybullet_envs.minitaur.agents.baseline_controller import locomotion_controller_setup
from pybullet_envs.minitaur.envs_v2 import env_loader
from pybullet_envs.minitaur.robots import robot_config

FLAGS = flags.FLAGS
flags.DEFINE_boolean("run_on_robot", False,
                     "whether to run in sim or on real hardware")
flags.DEFINE_boolean(
    "use_ground_truth_velocity", False,
    "whether to use a ground truth velocity estimator (available in sim)")
flags.DEFINE_enum("gait", "fast_trot",
                  ["fast_trot", "slow_trot", "walk", "stand"],
                  "The gait pattern to use")
flags.DEFINE_boolean(
    "use_keyboard_control", False,
    "whether to use a keyboard to control or demo speed profile.")
flags.DEFINE_string("log_path", None, "Path to save robot logs")
flags.DEFINE_boolean("add_random_push", False,
                     "whether to add random push to the robot in simulation")

_MAX_TIME_SECONDS = 100


def _load_config(render=True, run_on_robot=False):
  """Builds the environment for the quadruped robot.

  Args:
    render: Enable/disable rendering.
    run_on_robot: Whether deploy to robot or run in sim.
  """
  if run_on_robot:
    locomotion_controller_setup.load_real_config()
  else:
    locomotion_controller_setup.load_sim_config(render)
    if FLAGS.add_random_push:
      locomotion_controller_setup.add_random_push_config()


def _generate_example_linear_angular_speed(t):
  """Creates an example speed profile based on time for demo purpose."""
  vx = 0.1
  vy = 0.1
  wz = 0.3
  time_points = (0, 4, 7, 11, 13, 15, 17, 19, 100)
  speed_points = ((0, 0, 0, 0), (vx, 0, 0, 0), (-vx, 0, 0, 0), (0, -vy, 0, 0),
                  (0, vy, 0, 0), (0, 0, 0, wz), (0, 0, 0, -wz), (0, 0, 0, 0),
                  (0, 0, 0, 0))

  speed = scipy.interpolate.interp1d(
      time_points,
      speed_points,
      kind="previous",
      fill_value="extrapolate",
      axis=0)(
          t)

  return speed[0:3], speed[3]


def _update_speed_from_kb(kb, lin_speed, ang_speed):
  """Updates the controller behavior parameters."""
  if kb.is_keyboard_hit():
    c = kb.get_input_character()
    if c == "w":
      lin_speed += np.array((0.05, 0, 0))
    if c == "s":
      lin_speed += np.array((-0.05, 0, 0))
    if c == "q":
      ang_speed += 0.1
    if c == "e":
      ang_speed += -0.1
    if c == "a":
      lin_speed += np.array((0, 0.05, 0))
    if c == "d":
      lin_speed += np.array((0, -0.05, 0))
    if c == "r":
      lin_speed = np.array([0.0, 0.0, 0.0])
      ang_speed = 0.0

    lin_speed[0] = np.clip(lin_speed[0], -0.2, 0.4)
    lin_speed[1] = np.clip(lin_speed[1], -0.2, 0.2)
    ang_speed = np.clip(ang_speed, -0.3, 0.3)
    print("desired speed: ", lin_speed, ang_speed)

  return lin_speed, ang_speed


def _update_controller_params(controller, lin_speed, ang_speed):
  controller.swing_leg_controller.desired_speed = lin_speed
  controller.swing_leg_controller.desired_twisting_speed = ang_speed
  controller.stance_leg_controller.desired_speed = lin_speed
  controller.stance_leg_controller.desired_twisting_speed = ang_speed


def _run_example(max_time=_MAX_TIME_SECONDS,
                 run_on_robot=False,
                 use_keyboard=False):
  """Runs the locomotion controller example."""
  if use_keyboard:
    kb = keyboard_utils.KeyboardInput()

  env = env_loader.load()
  env.reset()

  # To mitigate jittering from the python
  gc.collect()

  # Wait for the robot to be placed properly.
  if run_on_robot:
    input("Press Enter to continue when robot is ready.")

  lin_speed = np.array([0.0, 0.0, 0.0])
  ang_speed = 0.0

  controller = locomotion_controller_setup.setup_controller(
      env.robot, FLAGS.gait, run_on_robot, FLAGS.use_ground_truth_velocity)
  controller.reset()

  loop_start_time = env.get_time_since_reset()
  loop_elapsed_time = 0
  robot_log = {
      "timestamps": [],
      "motor_angles": [],
      "motor_velocities": [],
      "base_velocities": [],
      "foot_positions": [],
      "base_rollpitchyaw": [],
      "base_angular_velocities": [],
      "actions": []
  }
  try:
    while loop_elapsed_time < max_time:
      #if use_keyboard:
      #  lin_speed, ang_speed = _update_speed_from_kb(kb, lin_speed, ang_speed)
      #else:
      lin_speed, ang_speed = _generate_example_linear_angular_speed(
            loop_elapsed_time)

      # Needed before every call to get_action().
      _update_controller_params(controller, lin_speed, ang_speed)
      controller.update()
      hybrid_action = controller.get_action()

      # Log the robot data.
      robot_log["timestamps"].append(env.robot.GetTimeSinceReset())
      robot_log["motor_angles"].append(env.robot.motor_angles)
      robot_log["motor_velocities"].append(env.robot.motor_velocities)
      robot_log["base_velocities"].append(
          controller.state_estimator.com_velocity_body_yaw_aligned_frame)
      robot_log["foot_positions"].append(env.robot.foot_positions())
      robot_log["base_rollpitchyaw"].append(env.robot.base_roll_pitch_yaw)
      robot_log["base_angular_velocities"].append(
          env.robot.base_roll_pitch_yaw_rate)
      robot_log["actions"].append(hybrid_action)

      env.step(hybrid_action)
      loop_elapsed_time = env.get_time_since_reset() - loop_start_time

  finally:
    if FLAGS.run_on_robot:
      # Apply zero torques to the robot.
      env.robot.apply_action(
          [0] * env.robot.num_motors,
          motor_control_mode=robot_config.MotorControlMode.TORQUE)
    if FLAGS.log_path:
      pickle.dump(robot_log, gfile.Open(FLAGS.log_path + "/robot.log", "wb"))


def main(argv):
  del argv
  _load_config(render=True, run_on_robot=FLAGS.run_on_robot)
  _run_example(
      run_on_robot=FLAGS.run_on_robot, use_keyboard=FLAGS.use_keyboard_control)


if __name__ == "__main__":
  app.run(main)
