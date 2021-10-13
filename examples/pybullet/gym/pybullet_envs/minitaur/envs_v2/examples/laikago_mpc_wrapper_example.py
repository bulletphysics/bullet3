# Lint as: python3
r"""An example where Laikago walks forward using mpc controller.

This script illustrates an example of MPCLocomotionWrapper class for
controlling a simulated Laikago robot to walk on a flat ground. The wrapped
environment takes as input the target local foothold locations and the desired
base pose. An MPC-based controller is executed internally to compute the
required forces to achieve the desired foothold position and base movement.


blaze run -c opt \
//robotics/reinforcement_learning/minitaur/envs_v2/examples\
:laikago_mpc_wrapper_example
"""
import os
import tempfile

from absl import app
from absl import flags
import gin
import numpy as np
import pybullet_data as pd

from pybullet_envs.minitaur.envs_v2 import env_loader

FLAGS = flags.FLAGS
flags.DEFINE_string("video_file", None, "The filename for saving the videos.")

CONFIG_FILE_SIM = (pd.getDataPath()+"/configs/laikago_mpc_two_camera_random_stepstone.gin")

NUM_STEPS = 100
ENABLE_RENDERING = True  # Will be disabled for tests
ENV_RANDOM_SEED = 100
DEFAULT_TARGET_FOOTHOLD = (0.05, 0.0, -0.01)
DEFAULT_BASE_VELOCITY = (0.0, 0.0)
DEFAULT_TWIST_SPEED = 0.0
DEFAULT_BODY_HEIGHT = 0.45
DEFAULT_ROLL_PITCH = (0.0, 0.0)
DEFAULT_SWING_HEIGHT = 0.07


def _build_env():
  """Builds the environment for the Laikago robot.

  Returns:
    The OpenAI gym environment.
  """
  gin.parse_config_file(CONFIG_FILE_SIM)
  gin.bind_parameter("SimulationParameters.enable_rendering", ENABLE_RENDERING)
  env = env_loader.load()
  env.seed(ENV_RANDOM_SEED)

  return env


def _run_example():
  """An example that Laikago moves with a constant speed and predicts foothold.

  Returns:
    env: the environment after the simulation
  """

  env = _build_env()

  env.reset()
  if FLAGS.video_file is not None:
    pybullet = env.pybullet_client
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
    log_id = pybullet.startStateLogging(pybullet.STATE_LOGGING_VIDEO_MP4,
                                        FLAGS.video_file)

  try:
    max_step = NUM_STEPS
    for _ in range(max_step):
      target_foothold = np.array(DEFAULT_TARGET_FOOTHOLD)
      action = np.concatenate([
          target_foothold, target_foothold, target_foothold, target_foothold,
          [
              DEFAULT_SWING_HEIGHT, DEFAULT_SWING_HEIGHT, DEFAULT_SWING_HEIGHT,
              DEFAULT_SWING_HEIGHT
          ], DEFAULT_BASE_VELOCITY, [DEFAULT_TWIST_SPEED],
          [DEFAULT_BODY_HEIGHT], DEFAULT_ROLL_PITCH
      ])
      _ = env.step(action)
  finally:
    if FLAGS.video_dir is not None:
      pybullet.stopStateLogging(log_id)


def main(argv):
  del argv
  _run_example()


if __name__ == "__main__":
  app.run(main)
