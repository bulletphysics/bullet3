import json
import numpy as np
import pybullet_utils.math_util as MathUtil

class ExpParams(object):
    RATE_KEY = 'Rate'
    INIT_ACTION_RATE_KEY = 'InitActionRate'
    NOISE_KEY = 'Noise'
    NOISE_INTERNAL_KEY = 'NoiseInternal'
    TEMP_KEY = 'Temp'

    def __init__(self):
        self.rate = 0.2
        self.init_action_rate = 0
        self.noise = 0.1
        self.noise_internal = 0
        self.temp = 0.1
        return

    def __str__(self):
        str = ''
        str += '{}: {:.2f}\n'.format(self.RATE_KEY, self.rate)
        str += '{}: {:.2f}\n'.format(self.INIT_ACTION_RATE_KEY, self.init_action_rate)
        str += '{}: {:.2f}\n'.format(self.NOISE_KEY, self.noise)
        str += '{}: {:.2f}\n'.format(self.NOISE_INTERNAL_KEY, self.noise_internal)
        str += '{}: {:.2f}\n'.format(self.TEMP_KEY, self.temp)
        return str

    def load(self, json_data):
        if (self.RATE_KEY in json_data):
            self.rate = json_data[self.RATE_KEY]

        if (self.INIT_ACTION_RATE_KEY in json_data):
            self.init_action_rate = json_data[self.INIT_ACTION_RATE_KEY]

        if (self.NOISE_KEY in json_data):
            self.noise = json_data[self.NOISE_KEY]

        if (self.NOISE_INTERNAL_KEY in json_data):
            self.noise_internal = json_data[self.NOISE_INTERNAL_KEY]

        if (self.TEMP_KEY in json_data):
            self.temp = json_data[self.TEMP_KEY]

        return

    def lerp(self, other, t):
        lerp_params = ExpParams()
        lerp_params.rate = MathUtil.lerp(self.rate, other.rate, t)
        lerp_params.init_action_rate = MathUtil.lerp(self.init_action_rate, other.init_action_rate, t)
        lerp_params.noise = MathUtil.lerp(self.noise, other.noise, t)
        lerp_params.noise_internal = MathUtil.lerp(self.noise_internal, other.noise_internal, t)
        lerp_params.temp = MathUtil.log_lerp(self.temp, other.temp, t)
        return lerp_params