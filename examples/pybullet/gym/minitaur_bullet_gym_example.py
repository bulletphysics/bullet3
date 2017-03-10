import gym
import numpy as np
import math

from envs.bullet.minitaur_bullet import MinitaurBulletEnv

def main():
    environment = gym.make('MinitaurBulletEnv-v0')
    sum_reward = 0
    steps = 1000
    amplitude = 0.5
    speed = 0.3

    for stepCounter in range(steps):
      a1 = math.sin(stepCounter*speed)*amplitude
      a2 = math.sin(stepCounter*speed+3.14)*amplitude
      action = [a1, 0, a2, 0, 0, a1, 0, a2]
      state, reward, done, info = environment.step(action)
      sum_reward += reward
      print(state)
      if done:
        environment.reset()
    average_reward = sum_reward / steps
    print("avg reward: ", average_reward)


main()
