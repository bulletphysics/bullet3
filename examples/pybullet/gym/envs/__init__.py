from gym.envs.registration import registry, register, make, spec

# ------------bullet-------------

register(
    id='CartPoleBulletEnv-v0',
    entry_point='envs.bullet:CartPoleBulletEnv',
    timestep_limit=1000,
    reward_threshold=950.0,
)

register(
    id='MinitaurBulletEnv-v0',
    entry_point='envs.bullet:MinitaurBulletEnv',
    timestep_limit=1000,
    reward_threshold=5.0,
)

register(
    id='RacecarBulletEnv-v0',
    entry_point='envs.bullet:RacecarBulletEnv',
    timestep_limit=1000,
    reward_threshold=5.0,
)

register(
    id='RacecarZedBulletEnv-v0',
    entry_point='envs.bullet:RacecarZEDGymEnv',
    timestep_limit=1000,
    reward_threshold=5.0,
)

register(
    id='HumanoidBulletEnv-v0',
    entry_point='envs.bullet:HumanoidGymEnv',
    timestep_limit=1000,
    reward_threshold=5.0,
)

register(
    id='KukaBulletEnv-v0',
    entry_point='envs.bullet:KukaGymEnv',
    timestep_limit=1000,
    reward_threshold=5.0,
)

register(
    id='KukaCamBulletEnv-v0',
    entry_point='envs.bullet:KukaCamGymEnv',
    timestep_limit=1000,
    reward_threshold=5.0,
)