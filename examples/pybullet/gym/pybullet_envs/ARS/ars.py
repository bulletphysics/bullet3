# AI 2018

import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)

# Importing the libraries
import os
import numpy as np
import gym
from gym import wrappers
import pybullet_envs
import time
import multiprocessing as mp
from multiprocessing import Process, Pipe
import argparse


# Setting the Hyper Parameters
class Hp():
    
    def __init__(self):
        self.nb_steps = 10000
        self.episode_length = 1000
        self.learning_rate = 0.02
        self.nb_directions = 16
        self.nb_best_directions = 16
        assert self.nb_best_directions <= self.nb_directions
        self.noise = 0.03
        self.seed = 1
        self.env_name = 'HalfCheetahBulletEnv-v0'


# Multiprocess Exploring the policy on one specific direction and over one episode

_RESET = 1
_CLOSE = 2
_EXPLORE = 3

def ExploreWorker(rank,childPipe, envname, args):
    env = gym.make(envname)
    nb_inputs = env.observation_space.shape[0]
    normalizer = Normalizer(nb_inputs)
    observation_n = env.reset()
    n=0
    while True:
      n+=1
      try:
        # Only block for short times to have keyboard exceptions be raised.
        if not childPipe.poll(0.001):
          continue
        message, payload = childPipe.recv()
      except (EOFError, KeyboardInterrupt):
        break
      if message == _RESET:
        observation_n = env.reset()
        childPipe.send(["reset ok"])
        continue
      if message == _EXPLORE:
        #normalizer = payload[0] #use our local normalizer
        policy = payload[1]
        hp = payload[2]
        direction = payload[3]
        delta = payload[4]
        state = env.reset()
        done = False
        num_plays = 0.
        sum_rewards = 0
        while not done and num_plays < hp.episode_length:
            normalizer.observe(state)
            state = normalizer.normalize(state)
            action = policy.evaluate(state, delta, direction,hp)
            state, reward, done, _ = env.step(action)
            reward = max(min(reward, 1), -1)
            sum_rewards += reward
            num_plays += 1
        childPipe.send([sum_rewards])
        continue
      if message == _CLOSE:
        childPipe.send(["close ok"])
        break
    childPipe.close()
        

# Normalizing the states

class Normalizer():
    
    def __init__(self, nb_inputs):
        self.n = np.zeros(nb_inputs)
        self.mean = np.zeros(nb_inputs)
        self.mean_diff = np.zeros(nb_inputs)
        self.var = np.zeros(nb_inputs)
    
    def observe(self, x):
        self.n += 1.
        last_mean = self.mean.copy()
        self.mean += (x - self.mean) / self.n
        self.mean_diff += (x - last_mean) * (x - self.mean)
        self.var = (self.mean_diff / self.n).clip(min = 1e-2)
    
    def normalize(self, inputs):
        obs_mean = self.mean
        obs_std = np.sqrt(self.var)
        return (inputs - obs_mean) / obs_std

# Building the AI

class Policy():
    def __init__(self, input_size, output_size, env_name, args):
        try:
          self.theta = np.load(args.policy)
        except:
          self.theta = np.zeros((output_size, input_size))
        self.env_name = env_name
        print("Starting policy theta=",self.theta)
    def evaluate(self, input, delta, direction, hp):
        if direction is None:
            return np.clip(self.theta.dot(input), -1.0, 1.0)
        elif direction == "positive":
            return np.clip((self.theta + hp.noise*delta).dot(input), -1.0, 1.0)
        else:
            return np.clip((self.theta - hp.noise*delta).dot(input), -1.0, 1.0)
    
    def sample_deltas(self):
        return [np.random.randn(*self.theta.shape) for _ in range(hp.nb_directions)]
    
    def update(self, rollouts, sigma_r, args):
        step = np.zeros(self.theta.shape)
        for r_pos, r_neg, d in rollouts:
            step += (r_pos - r_neg) * d
        self.theta += hp.learning_rate / (hp.nb_best_directions * sigma_r) * step
        timestr = time.strftime("%Y%m%d-%H%M%S")
        np.save(args.logdir+"/policy_"+self.env_name+"_"+timestr+".npy", self.theta)


# Exploring the policy on one specific direction and over one episode

def explore(env, normalizer, policy, direction, delta, hp):
    state = env.reset()
    done = False
    num_plays = 0.
    sum_rewards = 0
    while not done and num_plays < hp.episode_length:
        normalizer.observe(state)
        state = normalizer.normalize(state)
        action = policy.evaluate(state, delta, direction, hp)
        state, reward, done, _ = env.step(action)
        reward = max(min(reward, 1), -1)
        sum_rewards += reward
        num_plays += 1
    return sum_rewards

# Training the AI

def train(env, policy, normalizer, hp, parentPipes, args):
    
    for step in range(hp.nb_steps):
        
        # Initializing the perturbations deltas and the positive/negative rewards
        deltas = policy.sample_deltas()
        positive_rewards = [0] * hp.nb_directions
        negative_rewards = [0] * hp.nb_directions
        
        if parentPipes:
          for k in range(hp.nb_directions):
            parentPipe = parentPipes[k]
            parentPipe.send([_EXPLORE,[normalizer, policy, hp, "positive", deltas[k]]])
          for k in range(hp.nb_directions):
            positive_rewards[k] = parentPipes[k].recv()[0]
          
          for k in range(hp.nb_directions):
            parentPipe = parentPipes[k]
            parentPipe.send([_EXPLORE,[normalizer, policy, hp, "negative", deltas[k]]])
          for k in range(hp.nb_directions):
            negative_rewards[k] = parentPipes[k].recv()[0]
          
        else:
          # Getting the positive rewards in the positive directions
          for k in range(hp.nb_directions):
              positive_rewards[k] = explore(env, normalizer, policy, "positive", deltas[k], hp)
        
          
          # Getting the negative rewards in the negative/opposite directions
          for k in range(hp.nb_directions):
              negative_rewards[k] = explore(env, normalizer, policy, "negative", deltas[k], hp)
            
        
        # Gathering all the positive/negative rewards to compute the standard deviation of these rewards
        all_rewards = np.array(positive_rewards + negative_rewards)
        sigma_r = all_rewards.std()
        
        # Sorting the rollouts by the max(r_pos, r_neg) and selecting the best directions
        scores = {k:max(r_pos, r_neg) for k,(r_pos,r_neg) in enumerate(zip(positive_rewards, negative_rewards))}
        order = sorted(scores.keys(), key = lambda x:scores[x])[:hp.nb_best_directions]
        rollouts = [(positive_rewards[k], negative_rewards[k], deltas[k]) for k in order]
        
        # Updating our policy
        policy.update(rollouts, sigma_r, args)
        
        # Printing the final reward of the policy after the update
        reward_evaluation = explore(env, normalizer, policy, None, None, hp)
        print('Step:', step, 'Reward:', reward_evaluation)

# Running the main code

def mkdir(base, name):
    path = os.path.join(base, name)
    if not os.path.exists(path):
        os.makedirs(path)
    return path




if __name__ == "__main__":
    mp.freeze_support()

    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--env', help='Gym environment name', type=str, default='HalfCheetahBulletEnv-v0')
    parser.add_argument('--seed', help='RNG seed', type=int, default=1)
    parser.add_argument('--render', help='OpenGL Visualizer', type=int, default=0)
    parser.add_argument('--movie',help='rgb_array gym movie',type=int, default=0)
    parser.add_argument('--steps', help='Number of steps', type=int, default=10000)
    parser.add_argument('--policy', help='Starting policy file (npy)', type=str, default='')
    parser.add_argument('--logdir', help='Directory root to log policy files (npy)', type=str, default='.')
    parser.add_argument('--mp', help='Enable multiprocessing', type=int, default=1)
        
    args = parser.parse_args()
        
    hp = Hp()
    hp.env_name = args.env
    hp.seed = args.seed
    hp.nb_steps = args.steps
    print("seed = ", hp.seed)
    np.random.seed(hp.seed)

    parentPipes = None
    if args.mp:
      num_processes = hp.nb_directions
      processes = []
      childPipes = []
      parentPipes = []
      
      for pr in range (num_processes):
        parentPipe, childPipe = Pipe()
        parentPipes.append(parentPipe)
        childPipes.append(childPipe)
      
      for rank in range(num_processes):
          p = mp.Process(target=ExploreWorker, args=(rank,childPipes[rank], hp.env_name, args))
          p.start()
          processes.append(p)
        
    work_dir = mkdir('exp', 'brs')
    monitor_dir = mkdir(work_dir, 'monitor')
    env = gym.make(hp.env_name)
    if args.render:
      env.render(mode = "human")
    if args.movie:
      env = wrappers.Monitor(env, monitor_dir, force = True)
    nb_inputs = env.observation_space.shape[0]
    nb_outputs = env.action_space.shape[0]
    policy = Policy(nb_inputs, nb_outputs,hp.env_name, args)
    normalizer = Normalizer(nb_inputs)
    
    print("start training")
    train(env, policy, normalizer, hp, parentPipes, args)

    if args.mp:
      for parentPipe in parentPipes:
        parentPipe.send([_CLOSE,"pay2"])
      
      for p in processes:
        p.join()
