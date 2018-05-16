
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import functools
from pybullet_envs.minitaur.envs import minitaur_gym_env
from pybullet_envs.minitaur.envs import minitaur_reactive_env
from pybullet_envs.minitaur.envs.env_randomizers import minitaur_env_randomizer
from pybullet_envs.minitaur.envs.env_randomizers import minitaur_env_randomizer_from_config as randomizer_config_lib

MAX_LENGTH = 1000


def merge_two_dicts(x, y):
  """Given two dicts, merge them into a new dict as a shallow copy."""
  z = dict(x)
  z.update(y)
  return z


# The default configurations.
DEFAULT_CONFIG = dict(
    num_workers=8,
    num_directions=8,
    num_iterations=1000,
    deltas_used=8,
    step_size=0.02,
    delta_std=0.03,
    rollout_length=MAX_LENGTH,
    shift=0,
    seed=237,
    policy_type="linear",
    filter="MeanStdFilter",
)

# Configuration specific to minitaur_gym_env.MinitaurGymEnv class.
MINITAUR_GYM_CONFIG_ADDITIONS = dict(
    env=functools.partial(
        minitaur_gym_env.MinitaurGymEnv,
        urdf_version=minitaur_gym_env.DERPY_V0_URDF_VERSION,
        accurate_motor_model_enabled=True,
        motor_overheat_protection=True,
        pd_control_enabled=True,
        env_randomizer=None,#minitaur_env_randomizer.MinitaurEnvRandomizer(),
        render=False,
        num_steps_to_log=MAX_LENGTH))
MINITAUR_GYM_CONFIG = merge_two_dicts(DEFAULT_CONFIG,
                                      MINITAUR_GYM_CONFIG_ADDITIONS)

# Configuration specific to MinitaurReactiveEnv class.
MINITAUR_REACTIVE_CONFIG_ADDITIONS = dict(
    env=functools.partial(
        minitaur_reactive_env.MinitaurReactiveEnv,
        urdf_version=minitaur_gym_env.RAINBOW_DASH_V0_URDF_VERSION,
        energy_weight=0.005,
        accurate_motor_model_enabled=True,
        pd_latency=0.003,
        control_latency=0.02,
        motor_kd=0.015,
        remove_default_joint_damping=True,
        env_randomizer=None,
        render=False,
        num_steps_to_log=MAX_LENGTH))
MINITAUR_REACTIVE_CONFIG = merge_two_dicts(DEFAULT_CONFIG,
                                           MINITAUR_REACTIVE_CONFIG_ADDITIONS)

# Configuration specific to MinitaurReactiveEnv class with randomizer.
MINITAUR_REACTIVE_RANDOMIZER_CONFIG_ADDITIONS = dict(
    env=functools.partial(
        minitaur_reactive_env.MinitaurReactiveEnv,
        urdf_version=minitaur_gym_env.RAINBOW_DASH_V0_URDF_VERSION,
        energy_weight=0.005,
        accurate_motor_model_enabled=True,
        pd_latency=0.003,
        control_latency=0.02,
        motor_kd=0.015,
        remove_default_joint_damping=True,
        env_randomizer=randomizer_config_lib.MinitaurEnvRandomizerFromConfig(),
        render=False,
        num_steps_to_log=MAX_LENGTH))
MINITAUR_REACTIVE_RANDOMIZER_CONFIG = merge_two_dicts(
    DEFAULT_CONFIG, MINITAUR_REACTIVE_RANDOMIZER_CONFIG_ADDITIONS)
