from envs.bullet.cartpole_bullet import CartPoleBulletEnv
from rllab.algos.trpo import TRPO
from rllab.baselines.linear_feature_baseline import LinearFeatureBaseline
from rllab.envs.gym_env import GymEnv
from rllab.envs.normalized_env import normalize
from rllab.misc.instrument import stub, run_experiment_lite
from rllab.policies.gaussian_mlp_policy import GaussianMLPPolicy
import subprocess
import time

stub(globals())

env = normalize(GymEnv("CartPoleBulletEnv-v0"))

policy = GaussianMLPPolicy(
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
    # Uncomment both lines (this and the plot parameter below) to enable plotting
#    plot=True,
)

#cmdStartBulletServer=['~/Projects/rllab/bullet_examples/run_physics_server.sh']
#subprocess.Popen(cmdStartBulletServer, shell=True)
#time.sleep(1)


run_experiment_lite(
    algo.train(),
    # Number of parallel workers for sampling
    n_parallel=1,
    # Only keep the snapshot parameters for the last iteration
    snapshot_mode="last",
    # Specifies the seed for the experiment. If this is not provided, a random seed
    # will be used
    seed=1,
 #   plot=True,
)
