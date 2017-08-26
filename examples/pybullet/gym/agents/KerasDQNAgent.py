#!/usr/bin/env python

# based on https://github.com/matthiasplappert/keras-rl/blob/master/examples/dqn_cartpole.py
# with some extra arg parsing

from keras.models import Sequential # The Sequential model is a sequential, feed-forward stack of layers.
from keras.layers import Dense, Activation, Flatten # Different types of layers
from keras.optimizers import Adam # A special type of optimizer

from rl.agents.dqn import DQNAgent # The deep Q Learning Agent as described in https://www.cs.toronto.edu/~vmnih/docs/dqn.pdf
from rl.policy import BoltzmannQPolicy # Instead of random actions, we tend to pick actions that have generated rewards before. As time goes on we only focus on one or two actions in each state.
from rl.memory import SequentialMemory # A first-in-first-out type of memory to do the experience replay on


def add_opts(parser):
    pass

class KerasDQNAgent(object):
    '''
    classdocs
    '''


    def __init__(self, opts):
        self.metadata = {
            'discrete_actions' : True,
        }
        
        self.opts = opts
        
        
    def configure(self, observation_space_shape, nb_actions):
        # Next, we build a simple model.
        model = Sequential()
        model.add(Flatten(input_shape=(1,) + observation_space_shape)) # input layer
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
        self.agent = DQNAgent(enable_double_dqn = False, model=model, nb_actions=nb_actions, memory=memory, nb_steps_warmup=10,
               target_model_update=1e-2, policy=policy)
        self.agent.compile(Adam(lr=1e-3), metrics=['mae'])
        
    def train(self, env, nb_steps, visualize, verbosity):
        self.agent.fit(env, nb_steps=nb_steps, visualize=visualize, verbose=verbosity)
    
    def test(self, env, nb_episodes, visualize):
        self.agent.test(env, nb_episodes=nb_episodes, visualize=visualize)
    
    def load_weights(self, load_file):
        self.agent.load_weights(load_file)
    
    def save_weights(self, save_file, overwrite):
        self.agent.save_weights(save_file, overwrite)
