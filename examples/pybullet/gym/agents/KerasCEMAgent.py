#!/usr/bin/env python

# based on https://github.com/matthiasplappert/keras-rl/blob/master/examples/cem_cartpole.py
# with some extra arg parsing

from keras.models import Sequential # The Sequential model is a sequential, feed-forward stack of layers.
from keras.layers import Dense, Activation, Flatten # Different types of layers
from keras.optimizers import Adam # A special type of optimizer

from rl.agents.cem import CEMAgent # The cross-entropy method Learning Agent as described in http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.81.6579&rep=rep1&type=pdf
from rl.memory import EpisodeParameterMemory

def add_opts(parser):
    parser.add_argument('--model-type', type=int, default=1,
                      help="the dense-softmax-layer model (1) or the deep network (2)")


class KerasCEMAgent(object):
    '''
    classdocs
    '''
    
    def __init__(self, opts):
        self.metadata = {
            'discrete_actions' : True
        }
        
        self.opts = opts
        
    def configure(self, observation_space_shape, nb_actions):
        if self.opts.model_type == 1:
            # Option 1 : Simple model
            model = Sequential()
            model.add(Flatten(input_shape=(1,) + observation_space_shape))
            model.add(Dense(nb_actions))
            model.add(Activation('softmax'))
            print(model.summary())
        elif self.opts.model_type == 2:
            # Option 2: deep network
            model = Sequential()
            model.add(Flatten(input_shape=(1,) + observation_space_shape))
            model.add(Dense(16))
            model.add(Activation('relu'))
            model.add(Dense(16))
            model.add(Activation('relu'))
            model.add(Dense(16))
            model.add(Activation('relu'))
            model.add(Dense(nb_actions))
            model.add(Activation('softmax'))
            print(model.summary())


        # Finally, we configure and compile our agent. You can use every built-in Keras optimizer and
        # even the metrics!
        memory = EpisodeParameterMemory(limit=1000, window_length=1)
        
        self.agent = CEMAgent(model=model, nb_actions=nb_actions, memory=memory,
                       batch_size=50, nb_steps_warmup=2000, train_interval=50, elite_frac=0.05)
        self.agent.compile()
    
    def train(self, env, nb_steps, visualize, verbosity):
        # Okay, now it's time to learn something! We visualize the training here for show, but this
        # slows down training quite a lot. You can always safely abort the training prematurely using
        # Ctrl + C.
        self.agent.fit(env, nb_steps=nb_steps, visualize=visualize, verbose=verbosity)
        
    def test(self, env, nb_episodes, visualize):
        # Finally, evaluate our algorithm for 5 episodes.
        self.agent.test(env, nb_episodes=nb_episodes, visualize=visualize)
        
    def load_weights(self, load_file):
        self.agent.load_weights(load_file)
        
    def save_weights(self, save_file, overwrite):
        # After training is done, we save the best weights.
        self.agent.save_weights(save_file, overwrite=overwrite)
