# Code adapted from https://github.com/araffin/rl-baselines-zoo
# it requires stable-baselines to be installed
# Colab Notebook: https://colab.research.google.com/drive/1nZkHO4QTYfAksm9ZTaZ5vXyC7szZxC3F
# You can run it using: python -m pybullet_envs.stable_baselines.train --algo td3 --env HalfCheetahBulletEnv-v0
# Author: Antonin RAFFIN
# MIT License
import argparse

import pybullet_envs

import gym
import numpy as np
from stable_baselines import SAC, TD3
from stable_baselines.common.noise import NormalActionNoise
from stable_baselines.common.callbacks import EvalCallback, CheckpointCallback
from stable_baselines.common.vec_env import DummyVecEnv

from pybullet_envs.stable_baselines.utils import TimeFeatureWrapper


if __name__ == '__main__':
    parser = argparse.ArgumentParser("Train an RL agent using Stable Baselines")
    parser.add_argument('--algo', help='RL Algorithm (Soft Actor-Critic by default)', default='sac',
                        type=str, required=False, choices=['sac', 'td3'])
    parser.add_argument('--env', type=str, default='HalfCheetahBulletEnv-v0', help='environment ID')
    parser.add_argument('-n', '--n-timesteps', help='Number of training timesteps', default=int(1e6),
                        type=int)
    parser.add_argument('--save-freq', help='Save the model every n steps (if negative, no checkpoint)',
                    default=-1, type=int)
    args = parser.parse_args()

    env_id = args.env
    n_timesteps = args.n_timesteps
    save_path = '{}_{}'.format(args.algo, env_id)

    # Instantiate and wrap the environment
    env = TimeFeatureWrapper(gym.make(env_id))

    # Create the evaluation environment and callbacks
    eval_env = DummyVecEnv([lambda: TimeFeatureWrapper(gym.make(env_id))])

    callbacks = [EvalCallback(eval_env, best_model_save_path=save_path)]

    # Save a checkpoint every n steps
    if args.save_freq > 0:
        callbacks.append(CheckpointCallback(save_freq=args.save_freq, save_path=save_path,
                                            name_prefix='rl_model'))

    algo = {
        'sac': SAC,
        'td3': TD3
    }[args.algo]

    n_actions = env.action_space.shape[0]

    # Tuned hyperparameters from https://github.com/araffin/rl-baselines-zoo
    hyperparams = {
        'sac': dict(batch_size=256, gamma=0.98, policy_kwargs=dict(layers=[256, 256]),
                    learning_starts=10000, buffer_size=int(2e5), tau=0.01),

        'td3': dict(batch_size=100, policy_kwargs=dict(layers=[400, 300]),
                    learning_rate=1e-3, learning_starts=10000, buffer_size=int(1e6),
                    train_freq=1000, gradient_steps=1000,
                    action_noise=NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions)))
    }[args.algo]

    model = algo('MlpPolicy', env, verbose=1, **hyperparams)
    try:
        model.learn(n_timesteps, callback=callbacks)
    except KeyboardInterrupt:
        pass

    print("Saving to {}.zip".format(save_path))
    model.save(save_path)
