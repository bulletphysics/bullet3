#!/usr/bin/env python

# based on https://github.com/matthiasplappert/keras-rl/blob/master/examples/ddpg_pendulum.py
# with some extra arg parsing

from keras.models import Sequential, Model # The Sequential model is a sequential, feed-forward stack of layers.
from keras.layers import Dense, Activation, Flatten, Input, merge # Different types of layers
from keras.optimizers import Adam # A special type of optimizer

from rl.agents import DDPGAgent # The Deep Differential Policy Gradient Learning Agent as described in http://arxiv.org/abs/1509.02971
from rl.memory import SequentialMemory # A first-in-first-out type of memory to do the experience replay on
from rl.random import OrnsteinUhlenbeckProcess # a noise process

def add_opts(parser):
    pass

class KerasDDPGAgent(object):
    '''
    classdocs
    '''
    
    def __init__(self, opts):
        self.metadata = {
            'discrete_actions' : False
        }
        
        self.opts = opts
        
    def configure(self, observation_space_shape, nb_actions):
        # Next, we build a simple model.
        # actor network
        actor = Sequential()
        actor.add(Flatten(input_shape=(1,) + observation_space_shape))
        actor.add(Dense(16))
        actor.add(Activation('relu'))
        actor.add(Dense(16))
        actor.add(Activation('relu'))
        actor.add(Dense(16))
        actor.add(Activation('relu'))
        actor.add(Dense(nb_actions))
        actor.add(Activation('linear'))
        print(actor.summary())

        # critic network
        action_input = Input(shape=(nb_actions,), name='action_input')
        observation_input = Input(shape=(1,) + observation_space_shape, name='observation_input')
        flattened_observation = Flatten()(observation_input)
        x = merge([action_input, flattened_observation], mode='concat')
        x = Dense(32)(x)
        x = Activation('relu')(x)
        x = Dense(32)(x)
        x = Activation('relu')(x)
        x = Dense(32)(x)
        x = Activation('relu')(x)
        x = Dense(1)(x)
        x = Activation('linear')(x)
        critic = Model(input=[action_input, observation_input], output=x)
        print(critic.summary())

        # Finally, we configure and compile our agent. You can use every built-in Keras optimizer and
        # even the metrics!
        memory = SequentialMemory(limit=100000, window_length=1)
        random_process = OrnsteinUhlenbeckProcess(size=nb_actions, theta=.15, mu=0., sigma=.3)
        self.agent = DDPGAgent(nb_actions=nb_actions, actor=actor, critic=critic, critic_action_input=action_input,
                          memory=memory, nb_steps_warmup_critic=100, nb_steps_warmup_actor=100,
                          random_process=random_process, gamma=.99, target_model_update=1e-3)
        self.agent.compile(Adam(lr=.001, clipnorm=1.), metrics=['mae'])
        
    def train(self, env, nb_steps, visualize, verbosity):
        # Okay, now it's time to learn something! We visualize the training here for show, but this
        # slows down training quite a lot. You can always safely abort the training prematurely using
        # Ctrl + C.
        self.agent.fit(env, nb_steps=nb_steps, visualize=visualize, verbose=verbosity, nb_max_episode_steps=200)

    def test(self, env, nb_episodes, visualize):
        # Finally, evaluate our algorithm for 5 episodes.
        self.agent.test(env, nb_episodes=nb_episodes, visualize=visualize, nb_max_episode_steps=200)
        
    def load_weights(self, load_file):
        self.agent.load_weights(load_file)
        
    def save_weights(self, save_file, overwrite):
        self.agent.save_weights(save_file, overwrite=True)