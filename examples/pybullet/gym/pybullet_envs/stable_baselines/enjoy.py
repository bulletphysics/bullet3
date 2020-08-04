# Code adapted from https://github.com/araffin/rl-baselines-zoo
# it requires stable-baselines to be installed
# Colab Notebook: https://colab.research.google.com/drive/1nZkHO4QTYfAksm9ZTaZ5vXyC7szZxC3F
# You can run it using: python -m pybullet_envs.stable_baselines.enjoy --algo td3 --env HalfCheetahBulletEnv-v0
# Author: Antonin RAFFIN
# MIT License
import os
import time
import argparse
import multiprocessing

import gym
import numpy as np
import pybullet_envs

from stable_baselines import SAC, TD3
from pybullet_envs.stable_baselines.utils import TimeFeatureWrapper


if __name__ == '__main__':
    parser = argparse.ArgumentParser("Enjoy an RL agent trained using Stable Baselines")
    parser.add_argument('--algo', help='RL Algorithm (Soft Actor-Critic by default)', default='sac',
                        type=str, required=False, choices=['sac', 'td3'])
    parser.add_argument('--env', type=str, default='HalfCheetahBulletEnv-v0', help='environment ID')
    parser.add_argument('-n', '--n-episodes', help='Number of episodes', default=5,
                        type=int)
    parser.add_argument('--no-render', action='store_true', default=False,
                        help='Do not render the environment')
    parser.add_argument('--load-best', action='store_true', default=False,
                        help='Load best model instead of last model if available')
    args = parser.parse_args()

    env_id = args.env
    # Create an env similar to the training env
    env = TimeFeatureWrapper(gym.make(env_id))

    # Use SubprocVecEnv for rendering
    if not args.no_render:
      env.render(mode='human')

    algo = {
        'sac': SAC,
        'td3': TD3
    }[args.algo]

    # We assume that the saved model is in the same folder
    save_path = '{}_{}.zip'.format(args.algo, env_id)

    if not os.path.isfile(save_path) or args.load_best:
        print("Loading best model")
        # Try to load best model
        save_path = os.path.join('{}_{}'.format(args.algo, env_id), 'best_model.zip')


    # Load the saved model
    model = algo.load(save_path, env=env)

    try:
        # Use deterministic actions for evaluation
        episode_rewards, episode_lengths = [], []
        for _ in range(args.n_episodes):
            obs = env.reset()
            done = False
            episode_reward = 0.0
            episode_length = 0
            while not done:
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, done, _info = env.step(action)
                episode_reward += reward

                episode_length += 1
                if not args.no_render:
                    env.render(mode='human')
                    dt = 1. / 240.
                    time.sleep(dt)
            episode_rewards.append(episode_reward)
            episode_lengths.append(episode_length)
            print("Episode {} reward={}, length={}".format(len(episode_rewards), episode_reward, episode_length))

        mean_reward = np.mean(episode_rewards)
        std_reward = np.std(episode_rewards)

        mean_len, std_len = np.mean(episode_lengths), np.std(episode_lengths)

        print("==== Results ====")
        print("Episode_reward={:.2f} +/- {:.2f}".format(mean_reward, std_reward))
        print("Episode_length={:.2f} +/- {:.2f}".format(mean_len, std_len))
    except KeyboardInterrupt:
        pass

    # Close process
    env.close()