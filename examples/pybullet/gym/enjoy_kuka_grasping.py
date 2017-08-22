import gym
from pybullet_envs.bullet.kukaGymEnv import KukaGymEnv

from baselines import deepq


def main():
    
    env = KukaGymEnv(renders=True)
    act = deepq.load("kuka_model.pkl")
    print(act)
    while True:
        obs, done = env.reset(), False
        print("===================================")        
        print("obs")
        print(obs)
        episode_rew = 0
        while not done:
            env.render()
            obs, rew, done, _ = env.step(act(obs[None])[0])
            episode_rew += rew
        print("Episode reward", episode_rew)


if __name__ == '__main__':
    main()
