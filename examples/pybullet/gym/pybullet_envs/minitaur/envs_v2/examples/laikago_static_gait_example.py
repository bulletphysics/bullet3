# Lint as: python3
r"""An example that the Laikago walks forward using a static gait.

blaze run -c opt //robotics/reinforcement_learning/minitaur/envs_v2/examples:\
laikago_static_gait_example
"""
from absl import app
from absl import flags
import gin
from pybullet_envs.minitaur.agents.baseline_controller import static_gait_controller
from pybullet_envs.minitaur.envs_v2 import env_loader
import pybullet_data as pd

flags.DEFINE_bool("render", True, "Whether to render the example.")

FLAGS = flags.FLAGS
_CONFIG_FILE = pd.getDataPath()+"/configs/laikago_walk_static_gait.gin"
_NUM_STEPS = 10000
_ENV_RANDOM_SEED = 13


def _load_config(render=False):
  gin.parse_config_file(_CONFIG_FILE)
  gin.bind_parameter("SimulationParameters.enable_rendering", render)


def run_example(num_max_steps=_NUM_STEPS):
  """Runs the example.

  Args:
    num_max_steps: Maximum number of steps this example should run for.
  """
  env = env_loader.load()

  env.seed(_ENV_RANDOM_SEED)
  observation = env.reset()
  policy = static_gait_controller.StaticGaitController(env.robot)

  for _ in range(num_max_steps):
    action = policy.act(observation)
    _, _, done, _ = env.step(action)
    if done:
      break


def main(_):
  _load_config(FLAGS.render)
  run_example()


if __name__ == "__main__":
  app.run(main)
