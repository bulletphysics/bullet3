#!/usr/bin/env python

# copy pasta from https://github.com/matthiasplappert/keras-rl/blob/master/examples/dqn_cartpole.py
# with some extra arg parsing

import numpy as np
import gym
import time

from keras.models import Sequential # The Sequential model is a linear stack of layers.
from keras.layers import Dense, Activation, Flatten # Different types of layers
from keras.optimizers import Adam # A special type of optimizer

from rl.agents.dqn import DQNAgent # The deep Q Learning Agent as described in https://www.cs.toronto.edu/~vmnih/docs/dqn.pdf
from rl.policy import BoltzmannQPolicy # Instead of random actions, we tend to pick actions that have generated rewards before. As time goes on we only focus on one or two actions in each state.
from rl.memory import SequentialMemory # A first-in-first-out type of memory to do the experience replay on

import GymEnvs.CartPolev0Env as CartPolev0Env
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--num-train', type=int, default=100)
parser.add_argument('--num-eval', type=int, default=0)
parser.add_argument('--show-train', action='store_true')
parser.add_argument('--show-eval', action='store_true')
parser.add_argument('--load-file', type=str, default=None)
parser.add_argument('--save-file', type=str, default=None)
CartPolev0Env.add_opts(parser)
opts = parser.parse_args()
print "OPTS", opts

# Get the environment and extract the number of actions.
env = CartPolev0Env.CartPolev0(opts=opts, discrete_actions=True)
nb_actions = env.action_space.n

# initialize random seed
np.random.seed(int(time.time()))
env.seed(int(time.time()))

# Next, we build a simple model.
model = Sequential()
model.add(Flatten(input_shape=(1,) + env.observation_space.shape)) # input layer
model.add(Dense(32)) # Just your regular fully connected NN layer
model.add(Activation('tanh')) # tanh activation layer
model.add(Dense(16)) # more model capacity through fully connected NN layers
model.add(Activation('relu')) # Rectified Linear Units
model.add(Dense(16)) # more model capacity through fully connected NN layers
model.add(Activation('relu')) # Rectified Linear Units
model.add(Dense(nb_actions)) # fully connected NN layer with one output for each action
model.add(Activation('linear')) # we want linear activations in the end
print(model.summary())

memory = SequentialMemory(limit=50000, window_length=1)
policy = BoltzmannQPolicy()
dqn = DQNAgent(model=model, nb_actions=nb_actions, memory=memory, nb_steps_warmup=10,
               target_model_update=1e-2, policy=policy)
dqn.compile(Adam(lr=1e-3), metrics=['mae'])

if opts.load_file is not None:
  print "loading weights from from [%s]" % opts.load_file
  dqn.load_weights(opts.load_file)

# Okay, now it's time to learn something! We visualize the training here for show, but this
# slows down training quite a lot. You can always safely abort the training prematurely using
# Ctrl + C.
dqn.fit(env, nb_steps=opts.num_train, visualize=True, verbose=2)

# After training is done, we save the final weights.
if opts.save_file is not None:
  print "saving weights to [%s]" % opts.save_file
  dqn.save_weights(opts.save_file, overwrite=True)

# Finally, evaluate our algorithm for 5 episodes.
dqn.test(env, nb_episodes=opts.num_eval, visualize=opts.show_eval)

