import gym
from gym.envs.registration import registry, make, spec


def register(id, *args, **kvargs):
  if id in registry.keys():
    return
  else:
    return gym.envs.registration.register(id, *args, **kvargs)


# ------------bullet-------------

register(
    id='HumanoidDeepMimicBackflipBulletEnv-v1',
    entry_point='pybullet_envs.deep_mimic.gym_env:HumanoidDeepMimicBackflipBulletEnv',
    max_episode_steps=2000,
    reward_threshold=2000.0,
    order_enforce=False,
)

register(
    id='HumanoidDeepMimicWalkBulletEnv-v1',
    entry_point='pybullet_envs.deep_mimic.gym_env:HumanoidDeepMimicWalkBulletEnv',
    max_episode_steps=2000,
    reward_threshold=2000.0,
    order_enforce=False,
)

register(
    id='CartPoleBulletEnv-v1',
    entry_point='pybullet_envs.bullet:CartPoleBulletEnv',
    max_episode_steps=200,
    reward_threshold=190.0,
    order_enforce=False,
)

register(
    id='CartPoleContinuousBulletEnv-v0',
    entry_point='pybullet_envs.bullet:CartPoleContinuousBulletEnv',
    max_episode_steps=200,
    reward_threshold=190.0,
    order_enforce=False,
)


register(
    id='MinitaurBulletEnv-v0',
    entry_point='pybullet_envs.bullet:MinitaurBulletEnv',
    max_episode_steps=1000,
    reward_threshold=15.0,
    order_enforce=False,
)

register(
    id='MinitaurBulletDuckEnv-v0',
    entry_point='pybullet_envs.bullet:MinitaurBulletDuckEnv',
    max_episode_steps=1000,
    reward_threshold=5.0,
    order_enforce=False,
)

register(
    id='MinitaurExtendedEnv-v0',
    entry_point='pybullet_envs.minitaur.envs:MinitaurExtendedEnv',
    max_episode_steps=1000,
    reward_threshold=5.0,
    order_enforce=False,
)


register(
    id='MinitaurReactiveEnv-v0',
    entry_point='pybullet_envs.minitaur.envs:MinitaurReactiveEnv',
    max_episode_steps=1000,
    reward_threshold=5.0,
    order_enforce=False,
)

register(
    id='MinitaurBallGymEnv-v0',
    entry_point='pybullet_envs.minitaur.envs:MinitaurBallGymEnv',
    max_episode_steps=1000,
    reward_threshold=5.0,
    order_enforce=False,
)

register(
    id='MinitaurTrottingEnv-v0',
    entry_point='pybullet_envs.minitaur.envs:MinitaurTrottingEnv',
    max_episode_steps=1000,
    reward_threshold=5.0,
    order_enforce=False,
)

register(
    id='MinitaurStandGymEnv-v0',
    entry_point='pybullet_envs.minitaur.envs:MinitaurStandGymEnv',
    max_episode_steps=1000,
    reward_threshold=5.0,
    order_enforce=False,
)

register(
    id='MinitaurAlternatingLegsEnv-v0',
    entry_point='pybullet_envs.minitaur.envs:MinitaurAlternatingLegsEnv',
    max_episode_steps=1000,
    reward_threshold=5.0,
    order_enforce=False,
)

register(
    id='MinitaurFourLegStandEnv-v0',
    entry_point='pybullet_envs.minitaur.envs:MinitaurFourLegStandEnv',
    max_episode_steps=1000,
    reward_threshold=5.0,
    order_enforce=False,
)

register(
    id='RacecarBulletEnv-v0',
    entry_point='pybullet_envs.bullet:RacecarGymEnv',
    max_episode_steps=1000,
    reward_threshold=5.0,
    order_enforce=False,
)

register(
    id='RacecarZedBulletEnv-v0',
    entry_point='pybullet_envs.bullet:RacecarZEDGymEnv',
    max_episode_steps=1000,
    reward_threshold=5.0,
    order_enforce=False,
)

register(
    id='KukaBulletEnv-v0',
    entry_point='pybullet_envs.bullet:KukaGymEnv',
    max_episode_steps=1000,
    reward_threshold=5.0,
    order_enforce=False,
)

register(
    id='KukaCamBulletEnv-v0',
    entry_point='pybullet_envs.bullet:KukaCamGymEnv',
    max_episode_steps=1000,
    reward_threshold=5.0,
    order_enforce=False,
)

register(
    id='KukaDiverseObjectGrasping-v0',
    entry_point='pybullet_envs.bullet:KukaDiverseObjectEnv',
    max_episode_steps=1000,
    reward_threshold=5.0,
    order_enforce=False,
)

register(
    id='InvertedPendulumBulletEnv-v0',
    entry_point='pybullet_envs.gym_pendulum_envs:InvertedPendulumBulletEnv',
    max_episode_steps=1000,
    reward_threshold=950.0,
    order_enforce=False,
)

register(
    id='InvertedDoublePendulumBulletEnv-v0',
    entry_point='pybullet_envs.gym_pendulum_envs:InvertedDoublePendulumBulletEnv',
    max_episode_steps=1000,
    reward_threshold=9100.0,
    order_enforce=False,
)

register(
    id='InvertedPendulumSwingupBulletEnv-v0',
    entry_point='pybullet_envs.gym_pendulum_envs:InvertedPendulumSwingupBulletEnv',
    max_episode_steps=1000,
    reward_threshold=800.0,
    order_enforce=False,
)

register(
    id='ReacherBulletEnv-v0',
    entry_point='pybullet_envs.gym_manipulator_envs:ReacherBulletEnv',
    max_episode_steps=150,
    reward_threshold=18.0,
    order_enforce=False,
)

register(
    id='PusherBulletEnv-v0',
    entry_point='pybullet_envs.gym_manipulator_envs:PusherBulletEnv',
    max_episode_steps=150,
    reward_threshold=18.0,
    order_enforce=False,
)

register(
    id='ThrowerBulletEnv-v0',
    entry_point='pybullet_envs.gym_manipulator_envs:ThrowerBulletEnv',
    max_episode_steps=100,
    reward_threshold=18.0,
    order_enforce=False,
)

#register(
#    id='StrikerBulletEnv-v0',
#    entry_point='pybullet_envs.gym_manipulator_envs:StrikerBulletEnv',
#    max_episode_steps=100,
#    reward_threshold=18.0,
#    order_enforce=False,
#)

register(id='Walker2DBulletEnv-v0',
         entry_point='pybullet_envs.gym_locomotion_envs:Walker2DBulletEnv',
         max_episode_steps=1000,
         reward_threshold=2500.0,
         order_enforce=False,
)
register(id='HalfCheetahBulletEnv-v0',
         entry_point='pybullet_envs.gym_locomotion_envs:HalfCheetahBulletEnv',
         max_episode_steps=1000,
         reward_threshold=3000.0,
         order_enforce=False,
)

register(id='AntBulletEnv-v0',
         entry_point='pybullet_envs.gym_locomotion_envs:AntBulletEnv',
         max_episode_steps=1000,
         reward_threshold=2500.0,
         order_enforce=False,
)

register(id='HopperBulletEnv-v0',
         entry_point='pybullet_envs.gym_locomotion_envs:HopperBulletEnv',
         max_episode_steps=1000,
         reward_threshold=2500.0,
         order_enforce=False,
)

register(id='HumanoidBulletEnv-v0',
         entry_point='pybullet_envs.gym_locomotion_envs:HumanoidBulletEnv',
         max_episode_steps=1000,
         order_enforce=False,
)

register(id='HumanoidFlagrunBulletEnv-v0',
         entry_point='pybullet_envs.gym_locomotion_envs:HumanoidFlagrunBulletEnv',
         max_episode_steps=1000,
         reward_threshold=2000.0,
         order_enforce=False,
)

register(id='HumanoidFlagrunHarderBulletEnv-v0',
         entry_point='pybullet_envs.gym_locomotion_envs:HumanoidFlagrunHarderBulletEnv',
         max_episode_steps=1000,
         order_enforce=False,
)

#register(
#	id='AtlasBulletEnv-v0',
#	entry_point='pybullet_envs.gym_locomotion_envs:AtlasBulletEnv',
#	max_episode_steps=1000,
# order_enforce=False,
#	)


def getList():
  btenvs = ['- ' + spec.id for spec in gym.envs.registry.all() if spec.id.find('Bullet') >= 0]
  btenvs.extend([
        '- MinitaurExtendedEnv-v0', '- MinitaurReactiveEnv-v0',
        '- MinitaurBallGymEnv-v0', '- MinitaurTrottingEnv-v0',
        '- MinitaurStandGymEnv-v0', '- MinitaurAlternatingLegsEnv-v0',
        '- MinitaurFourLegStandEnv-v0', '- KukaDiverseObjectGrasping-v0'
    ])
  return btenvs
