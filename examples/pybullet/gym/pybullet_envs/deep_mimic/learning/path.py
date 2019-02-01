import numpy as np
from pybullet_envs.deep_mimic.env.env import Env

class Path(object):
    def __init__(self):
        self.clear()
        return

    def pathlength(self):
        return len(self.actions)

    def is_valid(self):
        valid = True
        l = self.pathlength()
        valid &= len(self.states) == l + 1
        valid &= len(self.goals) == l + 1
        valid &= len(self.actions) == l
        valid &= len(self.logps) == l
        valid &= len(self.rewards) == l
        valid &= len(self.flags) == l

        return valid

    def check_vals(self):
        for vals in [self.states, self.goals, self.actions, self.logps,
                  self.rewards]:
            for v in vals:
                if not np.isfinite(v).all():
                    return False
        return True

    def clear(self):
        self.states = []
        self.goals = []
        self.actions = []
        self.logps = []
        self.rewards = []
        self.flags = []
        self.terminate = Env.Terminate.Null
        return

    def get_pathlen(self):
        return len(self.rewards)

    def calc_return(self):
        return sum(self.rewards)