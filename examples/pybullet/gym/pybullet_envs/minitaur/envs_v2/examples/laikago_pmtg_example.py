r"""An example to run an OpenAI gym environment with laikago and PMTG wrapper.

This is an open-loop controller where we use 0 residuals and only execute the
trajectory generator. The parameters that are (normally modulated by the policy)
are fixed (except the intensity) and not optimized. They are hand picked as
follows:
 - The gait cycle frequency is 3 Hz.
 - Walking height is neutral (0).
 - Swing vs stance ratio is 2 (swing takes half the time vs stance).
 - Intensity starts from zero and is gradually increased over time.

blaze run -c opt \
//robotics/reinforcement_learning/minitaur/envs_v2/examples\
:laikago_pmtg_example
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os

import gin
import sys

from pybullet_envs.minitaur.envs_v2 import env_loader
import pybullet_data as pd

CONFIG_DIR = pd.getDataPath()+"/configs_v2/"
CONFIG_FILES = [
    os.path.join(CONFIG_DIR, "base/laikago_with_imu.gin"),
    os.path.join(CONFIG_DIR, "tasks/fwd_task_no_termination.gin"),
    os.path.join(CONFIG_DIR, "wrappers/pmtg_wrapper.gin"),
    os.path.join(CONFIG_DIR, "scenes/simple_scene.gin")
]
# Constants used to show example PMTG behavior with zero residual.
# Since we use the default PMTG configuration, there is only one trajectory
# generator. So the parameters that can be changed are:
# - Multiplier for delta time (similar to frequency but per time step).
# - Intensity of the trajectory generator.
# - Walking heights used for the legs.
# - The ratio of the speed of the leg during swing vs stance phase.
# For more details, check out TgSimple._process_tg_params method.
_PMTG_DELTA_TIME_MULTIPLIER = 2.0
_PMTG_INTENSITY_RANGE = (0.0, 1.5)
_PMTG_INTENSITY_STEP_SIZE = 0.0001
_PMTG_WALKING_HEIGHT = -0.3
_PMTG_SWING_VS_STANCE = 2
_NUM_MOTORS = 12


def main(argv):
  del argv  # Unused.

  # Parse the gym config and create the environment.
  for gin_file in CONFIG_FILES:
    gin.parse_config_file(gin_file)
  gin.bind_parameter("SimulationParameters.enable_rendering", True)
  gin.bind_parameter("terminal_conditions.maxstep_terminal_condition.max_step",
                     10000)
  env = env_loader.load()
  tg_intensity = _PMTG_INTENSITY_RANGE[0]
  sum_reward = 0
  env.reset()
  done = False
  # Use zero residual, only use the output of the trajectory generator.
  residual = [0] * _NUM_MOTORS
  # Since we fix residuals and all the parameters of the TG, this example
  # is practically an open loop controller. A learned policy would provide
  # different values for these parameters at every timestep.
  while not done:
    # Increase the intensity of the trajectory generator gradually
    # to illustrate increasingly larger steps.
    if tg_intensity < _PMTG_INTENSITY_RANGE[1]:
      tg_intensity += _PMTG_INTENSITY_STEP_SIZE
    tg_params = [
        _PMTG_DELTA_TIME_MULTIPLIER, tg_intensity, _PMTG_WALKING_HEIGHT,
        _PMTG_SWING_VS_STANCE
    ]
    action = residual + tg_params
    _, reward, done, _ = env.step(action)
    sum_reward += reward


if __name__ == "__main__":
  
  main(sys.argv)
