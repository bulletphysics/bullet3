"""One-line documentation for gym_example module.

A detailed description of gym_example.
"""

import gym
from envs.bullet.cartpole_bullet import CartPoleBulletEnv
import setuptools
import time
import numpy as np


w = [0.3, 0.02, 0.02, 0.012]

def main():
  env = gym.make('CartPoleBulletEnv-v0')
  for i_episode in range(1):
    observation = env.reset()
    done = False
    t = 0
    while not done:
        print(observation)
        action = np.array([np.inner(observation, w)])
        print(action)
        observation, reward, done, info = env.step(action)
        t = t + 1
        if done:
            print("Episode finished after {} timesteps".format(t+1))
            break

main()
