from envs.bullet.cartpole_bullet import CartPoleBulletEnv
from sandbox.rocky.tf.algos.trpo import TRPO
from sandbox.rocky.tf.policies.gaussian_mlp_policy import GaussianMLPPolicy
from sandbox.rocky.tf.envs.base import TfEnv

from rllab.baselines.linear_feature_baseline import LinearFeatureBaseline
from rllab.envs.gym_env import GymEnv
from rllab.envs.normalized_env import normalize

env = TfEnv(normalize(GymEnv("CartPoleBulletEnv-v0")))

policy = GaussianMLPPolicy(
    name = "tf_gaussian_mlp",
    env_spec=env.spec,
    # The neural network policy should have two hidden layers, each with 32 hidden units.
    hidden_sizes=(8,)
)

baseline = LinearFeatureBaseline(env_spec=env.spec)

algo = TRPO(
    env=env,
    policy=policy,
    baseline=baseline,
    batch_size=5000,
    max_path_length=env.horizon,
    n_itr=50,
    discount=0.999,
    step_size=0.01,
    force_batch_sampler=True,
    # Uncomment both lines (this and the plot parameter below) to enable plotting
    #plot=True,
)

algo.train()
