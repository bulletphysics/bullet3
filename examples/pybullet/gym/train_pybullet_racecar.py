import gym
from envs.bullet.racecarGymEnv import RacecarGymEnv

from baselines import deepq

import datetime



def callback(lcl, glb):
    # stop training if reward exceeds 199
    is_solved = lcl['t'] > 100 and sum(lcl['episode_rewards'][-101:-1]) / 100 >= 199
    #uniq_filename = "racecar_model" + str(datetime.datetime.now().date()) + '_' + str(datetime.datetime.now().time()).replace(':', '.')
    #print("uniq_filename=")
    #print(uniq_filename)
    #act.save(uniq_filename)
    return is_solved


def main():
  
    env = RacecarGymEnv(render=False)
    model = deepq.models.mlp([64])
    act = deepq.learn(
        env,
        q_func=model,
        lr=1e-3,
        max_timesteps=10000000,
        buffer_size=50000,
        exploration_fraction=0.1,
        exploration_final_eps=0.02,
        print_freq=10,
        callback=callback
    )
    print("Saving model to racecar_model.pkl")
    act.save("racecar_model.pkl")


if __name__ == '__main__':
    main()
