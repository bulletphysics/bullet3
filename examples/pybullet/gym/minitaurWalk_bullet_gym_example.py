"""One-line documentation for gym_example module.

A detailed description of gym_example.
"""

import gym
from envs.bullet.minitaur import MinitaurWalkEnv
import setuptools
import time
import numpy as np


def main():
  env = gym.make('MinitaurWalkEnv-v0')
  for i_episode in range(1):
    observation = env.reset()
    done = False
    while not done:
        print(observation)
        action = np.array([1.3, 0, 0, 0, 1.3, 0, 0, 0, 1.3, 3.14, 0, 0, 1.3, 3.14, 0, 0])
        print(action)
        observation, reward, done, info = env.step(action)
        if done:
            print("Episode finished after {} timesteps".format(t+1))
            break

main()
