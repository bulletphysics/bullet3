import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)
import pybullet_envs
import gym
import argparse
import pybullet as p


def test(args):
	count = 0
	env = gym.make(args.env)
	env.env.configure(args)
	print("args.render=",args.render)
	if (args.render==1):
		env.render(mode="human")
	env.reset()
	if (args.resetbenchmark):
		while (1):
			env.reset()
			print("p.getNumBodies()=",p.getNumBodies())
			print("count=",count)
			count+=1
	print("action space:")
	sample = env.action_space.sample()
	action = sample*0.0
	print("action=")
	print(action)
	for i in range(args.steps):
		obs,rewards,done,_ =env.step(action)
		if (args.rgb):
			print(env.render(mode="rgb_array"))
		print("obs=")
		print(obs)
		print("rewards")
		print (rewards)
		print ("done")
		print(done)


def main():
    import argparse
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--env', help='environment ID', default='AntBulletEnv-v0')
    parser.add_argument('--seed', help='RNG seed', type=int, default=0)
    parser.add_argument('--render', help='OpenGL Visualizer', type=int, default=0)
    parser.add_argument('--rgb',help='rgb_array gym rendering',type=int, default=0)
    parser.add_argument('--resetbenchmark',help='Repeat reset to show reset performance',type=int, default=0)
    parser.add_argument('--steps', help='Number of steps', type=int, default=1)
    
    args = parser.parse_args()
    test(args)

if __name__ == '__main__':
    main()
