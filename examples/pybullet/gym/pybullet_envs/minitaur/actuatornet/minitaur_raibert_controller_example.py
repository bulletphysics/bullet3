#The example to run the raibert controller in a Minitaur gym env.

#blaze run :minitaur_raibert_controller_example  -- --log_path=/tmp/logs

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import tensorflow as tf
from pybullet_envs.minitaur.envs import minitaur_raibert_controller
from pybullet_envs.minitaur.envs import minitaur_gym_env

flags = tf.app.flags
FLAGS = tf.app.flags.FLAGS

flags.DEFINE_float("motor_kp", 1.0, "The position gain of the motor.")
flags.DEFINE_float("motor_kd", 0.015, "The speed gain of the motor.")
flags.DEFINE_float(
    "control_latency", 0.006, "The latency between sensor measurement and action"
    " execution the robot.")
flags.DEFINE_string("log_path", ".", "The directory to write the log file.")


def speed(t):
  max_speed = 0.35
  t1 = 3
  if t < t1:
    return t / t1 * max_speed
  else:
    return -max_speed


def main(argv):
    del argv
    env = minitaur_gym_env.MinitaurGymEnv(
        urdf_version=minitaur_gym_env.RAINBOW_DASH_V0_URDF_VERSION,
        control_time_step=0.006,
        action_repeat=6,
        pd_latency=0.0,
        control_latency=FLAGS.control_latency,
        motor_kp=FLAGS.motor_kp,
        motor_kd=FLAGS.motor_kd,
        remove_default_joint_damping=True,
        leg_model_enabled=False,
        render=True,
        on_rack=False,
        accurate_motor_model_enabled=True,
        log_path=FLAGS.log_path)
    env.reset()

    controller = minitaur_raibert_controller.MinitaurRaibertTrottingController(
        env.minitaur)

    tstart = env.minitaur.GetTimeSinceReset()
    for _ in range(1000):
      t = env.minitaur.GetTimeSinceReset() - tstart
      controller.behavior_parameters = (
          minitaur_raibert_controller.BehaviorParameters(
              desired_forward_speed=speed(t)))
      controller.update(t)
      env.step(controller.get_action())

    #env.close()

if __name__ == "__main__":
  tf.app.run(main)

