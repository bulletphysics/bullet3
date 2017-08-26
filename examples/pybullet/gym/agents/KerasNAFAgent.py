#!/usr/bin/env python

# based on https://github.com/matthiasplappert/keras-rl/blob/master/examples/cdqn_pendulum.py
# with some extra arg parsing

from keras.models import Sequential, Model # The Sequential model is a sequential, feed-forward stack of layers.
from keras.layers import Dense, Activation, Flatten, Input, merge # Different types of layers
from keras.optimizers import Adam # A special type of optimizer

from rl.agents import ContinuousDQNAgent # The Normalized Advantage Functions Agent as described in https://arxiv.org/abs/1603.00748
from rl.memory import SequentialMemory # A first-in-first-out type of memory to do the experience replay on
from rl.random import OrnsteinUhlenbeckProcess # a noise process

def add_opts(parser):
    pass

class KerasNAFAgent(object):
    '''
    classdocs
    '''
    
    def __init__(self, opts):
        self.metadata = {
            'discrete_actions' : False,
        }
        
        self.opts = opts
        
    def configure(self, observation_space_shape, nb_actions):

        # Build all necessary models: V, mu, and L networks.
        V_model = Sequential()
        V_model.add(Flatten(input_shape=(1,) + observation_space_shape))
        V_model.add(Dense(16))
        V_model.add(Activation('relu'))
        V_model.add(Dense(16))
        V_model.add(Activation('relu'))
        V_model.add(Dense(16))
        V_model.add(Activation('relu'))
        V_model.add(Dense(1))
        V_model.add(Activation('linear'))
        print(V_model.summary())
        
        mu_model = Sequential()
        mu_model.add(Flatten(input_shape=(1,) + observation_space_shape))
        mu_model.add(Dense(16))
        mu_model.add(Activation('relu'))
        mu_model.add(Dense(16))
        mu_model.add(Activation('relu'))
        mu_model.add(Dense(16))
        mu_model.add(Activation('relu'))
        mu_model.add(Dense(nb_actions))
        mu_model.add(Activation('linear'))
        print(mu_model.summary())
        
        action_input = Input(shape=(nb_actions,), name='action_input')
        observation_input = Input(shape=(1,) + observation_space_shape, name='observation_input')
        x = merge([action_input, Flatten()(observation_input)], mode='concat')
        x = Dense(32)(x)
        x = Activation('relu')(x)
        x = Dense(32)(x)
        x = Activation('relu')(x)
        x = Dense(32)(x)
        x = Activation('relu')(x)
        x = Dense(((nb_actions * nb_actions + nb_actions) / 2))(x)
        x = Activation('linear')(x)
        L_model = Model(input=[action_input, observation_input], output=x)
        print(L_model.summary())

        # Finally, we configure and compile our agent. You can use every built-in Keras optimizer and
        # even the metrics!
        memory = SequentialMemory(limit=100000, window_length=1)
        random_process = OrnsteinUhlenbeckProcess(theta=.15, mu=0., sigma=.3, size=nb_actions)
        self.agent = ContinuousDQNAgent(nb_actions=nb_actions, V_model=V_model, L_model=L_model, mu_model=mu_model,
                                   memory=memory, nb_steps_warmup=100, random_process=random_process,
                                   gamma=.99, target_model_update=1e-3)
        self.agent.compile(Adam(lr=.001, clipnorm=1.), metrics=['mae'])
        
    def train(self, env, nb_steps, visualize, verbosity):
        # Okay, now it's time to learn something! We visualize the training here for show, but this
        # slows down training quite a lot. You can always safely abort the training prematurely using
        # Ctrl + C.
        self.agent.fit(env, nb_steps=nb_steps, visualize=visualize, verbose=verbosity, nb_max_episode_steps=200)
        
    def test(self, env, nb_episodes, visualize):
        self.agent.test(env, nb_episodes=nb_episodes, visualize=visualize, nb_max_episode_steps=200)
        
    def load_weights(self, load_file):
        self.agent.load_weights(load_file)
    
    def save_weights(self, save_file, overwrite):
        # After training is done, we save the final weights.
        self.agent.save_weights(save_file, overwrite=overwrite)


