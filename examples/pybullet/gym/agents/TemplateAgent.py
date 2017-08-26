#!/usr/bin/env python


def add_opts(parser):
    pass

class TemplateAgent(object):
    '''
    classdocs
    '''


    def __init__(self, opts):
        self.metadata = {
            'discrete_actions' : True,
        }
        
        self.opts = opts
        
        
    def configure(self, observation_space_shape, nb_actions):
        pass
        
    def train(self, env, nb_steps, visualize, verbosity):
        pass
    
    def test(self, env, nb_episodes, visualize):
        pass
    
    def load_weights(self, load_file):
        pass
    
    def save_weights(self, save_file, overwrite):
        pass