'''
A test for minitaurGymEnv
'''

import gym
import numpy as np
import math

import numpy as np
import tensorflow as tf

from envs.bullet.minitaurGymEnv import MinitaurGymEnv

try:
  import sonnet
  from agents import simpleAgentWithSonnet as agent_lib
  ckpt_path = 'data/agent/tf_graph_data/tf_graph_data_converted.ckpt-0'
except ImportError:
  from agents import simpleAgent as agent_lib
  ckpt_path = 'data/agent/tf_graph_data/tf_graph_data.ckpt'
  
def testSinePolicy():
  """Tests sine policy
    """
  np.random.seed(47)

  environment = MinitaurGymEnv(render=True)
  sum_reward = 0
  steps = 1000
  amplitude1Bound = 0.5
  amplitude2Bound = 0.15
  speed = 40

  for stepCounter in range(steps):
    t = float(stepCounter) * environment._timeStep

    if (t < 1):
      amplitude1 = 0
      amplitude2 = 0
    else:
      amplitude1 = amplitude1Bound
      amplitude2 = amplitude2Bound
    a1 = math.sin(t*speed)*amplitude1
    a2 = math.sin(t*speed+3.14)*amplitude1
    a3 = math.sin(t*speed)*amplitude2
    a4 = math.sin(t*speed+3.14)*amplitude2

    action = [a1, a2, a2, a1, a3, a4, a4, a3]

    state, reward, done, info = environment.step(action)
    sum_reward += reward
    if done:
      environment.reset()
      print("sum reward: ", sum_reward)


def testDDPGPolicy():
  """Tests sine policy
    """
  environment = MinitaurGymEnv(render=True)
  sum_reward = 0
  steps = 1000

  observation_shape = (31,)
  action_size = 8
  actor_layer_size = (100, 181)
  n_steps = 0
  tf.reset_default_graph()
  with tf.Session() as session:
    agent = agent_lib.SimpleAgent(session=session, ckpt_path=ckpt_path, actor_layer_size=actor_layer_size)
    state = environment.reset()
    action = agent(state)
    for _ in range(steps):
      n_steps += 1
      state, reward, done, info = environment.step(action)
      action = agent(state)
      sum_reward += reward
      if done:
        environment.reset()
        n_steps += 1
        print("total reward: ", sum_reward)
        print("total steps:  ", n_steps)
        sum_reward = 0
        n_steps = 0
        return


testDDPGPolicy()
#testSinePolicy()
