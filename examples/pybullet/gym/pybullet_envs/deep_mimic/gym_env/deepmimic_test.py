import gym
import pybullet_envs
import time

#env = gym.make('HumanoidDeepMimicBackflipBulletEnv-v1')
env = gym.make('HumanoidDeepMimicWalkBulletEnv-v1')
env.render(mode='human')
env.reset()
print("------------------------------------")
print("env=",env)
print(dir(env))
print(dir(env.env))
dt = 1./240.
logId = env.env._internal_env._pybullet_client.startStateLogging(env.env._internal_env._pybullet_client.STATE_LOGGING_PROFILE_TIMINGS, "perf.json")
for i in range (100):
  env.env._internal_env._pybullet_client.submitProfileTiming("loop")
  
  env.reset()
  env.env._internal_env._pybullet_client.submitProfileTiming()
env.env._internal_env._pybullet_client.stopStateLogging(logId)

action = env.env.action_space.sample()
while (1):
  time.sleep(dt)
  #keys = env.env._internal_env._pybullet_client.getKeyboardEvents()
  #if keys:
  #  env.reset()
  #action=[0]*36
  action = env.env.action_space.sample()
  state, reward, done, info = env.step(action)
  #env.render(mode='rgb_array')
  if done:
    env.reset()
    #action = env.env.action_space.sample()
  #print("reward=",reward)
    