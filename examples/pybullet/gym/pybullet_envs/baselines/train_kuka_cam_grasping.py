#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)

import gym
from pybullet_envs.bullet.kukaCamGymEnv import KukaCamGymEnv

from baselines import deepq

import datetime



def callback(lcl, glb):
    # stop training if reward exceeds 199
    total = sum(lcl['episode_rewards'][-101:-1]) / 100
    totalt = lcl['t']
    #print("totalt")
    #print(totalt)
    is_solved = totalt > 2000 and total >= 10
    return is_solved


def main():
  	
    env = KukaCamGymEnv(renders=False, isDiscrete=True)
    model = deepq.models.cnn_to_mlp(
        convs=[(32, 8, 4), (64, 4, 2), (64, 3, 1)],
        hiddens=[256],
        dueling=False
    )
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
    print("Saving model to kuka_cam_model.pkl")
    act.save("kuka_cam_model.pkl")


if __name__ == '__main__':
    main()
