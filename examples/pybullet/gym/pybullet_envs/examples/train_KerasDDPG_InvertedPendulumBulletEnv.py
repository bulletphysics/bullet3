#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)

import gym
import numpy as np
import pybullet as p
import pybullet_envs
import time
import datetime
import argparse
from pybullet_envs import Trainer


trainer = Trainer.Trainer()

argparser = argparse.ArgumentParser()
Trainer.add_opts(argparser)

# precoded options
opts = argparser.parse_args()
opts.agent = "KerasDDPGAgent-v0"
opts.env = "InvertedPendulumBulletEnv-v0"
opts.train_for = 10000000
opts.test_for = 0
datenow = '{:%Y%m%d%H%M%S}'.format(datetime.datetime.now())
opts.save_file = "checkpoints/%s-%s-%s.h5" % (opts.agent, opts.env, datenow)

print("\n OPTS", opts)

trainer.setup_exercise(opts)


