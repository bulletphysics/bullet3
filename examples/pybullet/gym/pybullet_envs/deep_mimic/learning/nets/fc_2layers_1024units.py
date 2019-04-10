import tensorflow as tf
import pybullet_envs.deep_mimic.learning.tf_util as TFUtil

NAME = "fc_2layers_1024units"

def build_net(input_tfs, reuse=False):
    layers = [1024, 512]
    activation = tf.nn.relu

    input_tf = tf.concat(axis=-1, values=input_tfs)          
    h = TFUtil.fc_net(input_tf, layers, activation=activation, reuse=reuse)
    h = activation(h)
    return h