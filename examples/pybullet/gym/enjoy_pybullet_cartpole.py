import gym

from baselines import deepq
from envs.bullet.cartpole_bullet import CartPoleBulletEnv

def main():
    env = gym.make('CartPoleBulletEnv-v0')
    act = deepq.load("cartpole_model.pkl")

    while True:
        obs, done = env.reset(), False
        print("obs")
        print(obs)
        print("type(obs)")
        print(type(obs))
        episode_rew = 0
        while not done:
            env.render()
           
            o = obs[None]
            aa = act(o)
            a = aa[0]
            obs, rew, done, _ = env.step(a)
            episode_rew += rew
        print("Episode reward", episode_rew)


if __name__ == '__main__':
    main()
