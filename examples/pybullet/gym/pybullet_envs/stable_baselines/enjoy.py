# Code adapted from https://github.com/DLR-RM/rl-baselines3-zoo
# it requires stable-baselines3 to be installed
# Colab Notebook: https://colab.research.google.com/github/Stable-Baselines-Team/rl-colab-notebooks/blob/sb3/pybullet.ipynb
# You can run it using: python -m pybullet_envs.stable_baselines.enjoy --algo td3 --env HalfCheetahBulletEnv-v0
# Author: Antonin RAFFIN
# MIT License
import os
import time
import argparse

import gym
import numpy as np
import pybullet_envs

from stable_baselines3 import SAC, TD3


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        "Enjoy an RL agent trained using Stable Baselines3"
    )
    parser.add_argument(
        "--algo",
        help="RL Algorithm (Soft Actor-Critic by default)",
        default="sac",
        type=str,
        required=False,
        choices=["sac", "td3"],
    )
    parser.add_argument(
        "--env", type=str, default="HalfCheetahBulletEnv-v0", help="environment ID"
    )
    parser.add_argument(
        "-n", "--n-episodes", help="Number of episodes", default=5, type=int
    )
    parser.add_argument(
        "--no-render",
        action="store_true",
        default=False,
        help="Do not render the environment",
    )
    parser.add_argument(
        "--load-best",
        action="store_true",
        default=False,
        help="Load best model instead of last model if available",
    )
    args = parser.parse_args()

    env_id = args.env
    # Create an env similar to the training env
    env = gym.make(env_id)

    # Enable GUI
    if not args.no_render:
        env.render(mode="human")

    algo = {
        "sac": SAC,
        "td3": TD3,
    }[args.algo]

    # We assume that the saved model is in the same folder
    save_path = f"{args.algo}_{env_id}.zip"

    if not os.path.isfile(save_path) or args.load_best:
        print("Loading best model")
        # Try to load best model
        save_path = os.path.join(f"{args.algo}_{env_id}", "best_model.zip")

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
                    env.render(mode="human")
                    dt = 1.0 / 240.0
                    time.sleep(dt)
            episode_rewards.append(episode_reward)
            episode_lengths.append(episode_length)
            print(
                f"Episode {len(episode_rewards)} reward={episode_reward}, length={episode_length}"
            )

        mean_reward = np.mean(episode_rewards)
        std_reward = np.std(episode_rewards)

        mean_len, std_len = np.mean(episode_lengths), np.std(episode_lengths)

        print("==== Results ====")
        print(f"Episode_reward={mean_reward:.2f} +/- {std_reward:.2f}")
        print(f"Episode_length={mean_len:.2f} +/- {std_len:.2f}")
    except KeyboardInterrupt:
        pass

    # Close process
    env.close()
