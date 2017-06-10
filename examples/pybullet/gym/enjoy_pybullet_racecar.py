import gym
from envs.bullet.racecarGymEnv import RacecarGymEnv

from baselines import deepq


def main():
    
    env = RacecarGymEnv(render=True)
    act = deepq.load("racecar_model.pkl")
    print(act)
    while True:
        obs, done = env.reset(), False
        print("===================================")        
        print("obs")
        print(obs)
        episode_rew = 0
        while not done:
            #env.render()

            print("!!!!!!!!!!!!!!!!!!!!!!!!!!")
            print("obs")
            print(obs)
            print("???????????????????????????")
            print("obs[None]")
            print(obs[None])
            o = obs[None]
            print("o")
            print(o)
            aa = act(o)
            print("aa")
            print (aa)
            a = aa[0]
            print("a")
            print(a)
            obs, rew, done, _ = env.step(a)
            print("===================================")
            print("obs")
            print(obs)
            episode_rew += rew
        print("Episode reward", episode_rew)


if __name__ == '__main__':
    main()
