import gym
import pybullet_envs
import time

env = gym.make('HumanoidDeepMimicBackflipBulletEnv-v1')
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
  #time.sleep(dt)
  #keys = env.env._internal_env._pybullet_client.getKeyboardEvents()
  #if keys:
  #  print (keys)
  env.reset()
  env.env._internal_env._pybullet_client.submitProfileTiming()

env.env._internal_env._pybullet_client.stopStateLogging(logId)