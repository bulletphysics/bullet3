import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)
import pybullet_envs
import gym
import argparse
import pybullet as p


def test(args):
  count = 0
  env = gym.make(args.env)
  env.env.configure(args)
  
  print("args.render=", args.render)
  if (args.render == 1):
    env.render(mode="human")
  env.reset()
  
  w, h, vmat,projmat,camup,camfwd,hor,ver,yaw,pitch,dist,target= p.getDebugVisualizerCamera()
  dist = 0.4
  yaw = 0
  p.resetDebugVisualizerCamera(dist,yaw, pitch,target)
  
  for obindex in range (p.getNumBodies()):
    obuid = p.getBodyUniqueId(obindex)
    p.changeDynamics(obuid, -1, linearDamping=0, angularDamping=0)
    for l in range (p.getNumJoints(obuid)):
      p.changeDynamics(obuid, l, linearDamping=0, angularDamping=0, jointDamping=0)
      #if (l==0):
      #  p.setJointMotorControl2(obuid,l,p.POSITION_CONTROL,targetPosition=0)
      if (l==2):
        jp,jv,_,_ = p.getJointState(obuid,l)
        p.resetJointState(obuid,l, jp,0.01 )
    
  if (args.resetbenchmark):
    while (1):
      env.reset()
      print("p.getNumBodies()=", p.getNumBodies())
      print("count=", count)
      count += 1
  print("action space:")
  sample = env.action_space.sample()
  action = sample * 0.0
  action = [0,0]#sample * 0.0
  
  print("action=")
  print(action)
  for i in range(args.steps):
    obs, rewards, done, _ = env.step(action)
    if (args.rgb):
      print(env.render(mode="rgb_array"))
    print("obs=")
    print(obs)
    print("rewards")
    print(rewards)
    print("done")
    print(done)


def main():
  import argparse
  parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('--env', help='environment ID', default='AntBulletEnv-v0')
  parser.add_argument('--seed', help='RNG seed', type=int, default=0)
  parser.add_argument('--render', help='OpenGL Visualizer', type=int, default=0)
  parser.add_argument('--rgb', help='rgb_array gym rendering', type=int, default=0)
  parser.add_argument('--resetbenchmark',
                      help='Repeat reset to show reset performance',
                      type=int,
                      default=0)
  parser.add_argument('--steps', help='Number of steps', type=int, default=1)

  args = parser.parse_args()
  test(args)


if __name__ == '__main__':
  main()
