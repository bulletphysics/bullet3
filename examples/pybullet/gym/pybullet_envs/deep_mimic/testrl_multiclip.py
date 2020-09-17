import time
import os
import inspect

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)
print("parentdir=", parentdir)
import json
from pybullet_envs.deep_mimic.learning.rl_world import RLWorld
from pybullet_envs.deep_mimic.learning.ppo_agent import PPOAgent

import pybullet_data
from pybullet_utils.arg_parser import ArgParser
from pybullet_utils.logger import Logger
from pybullet_envs.deep_mimic.env.pybullet_deep_mimic_env_multiclip import PyBulletDeepMimicEnvMultiClip
import sys
import random

update_timestep = 1. / 240.
animating = True
step = False
total_reward = 0
steps = 0


def update_world(world, time_elapsed):
    timeStep = update_timestep
    world.update(timeStep)
    reward = world.env.calc_reward(agent_id=0)
    global total_reward
    total_reward += reward
    global steps
    steps += 1

    end_episode = world.env.is_episode_end()
    if (end_episode or steps >= 1000):
        print("total_reward=", total_reward)
        total_reward = 0
        steps = 0
        world.end_episode()
        world.reset()


def build_arg_parser(args):
    arg_parser = ArgParser()
    arg_parser.load_args(args)

    arg_file = arg_parser.parse_string('arg_file', '')
    if arg_file == '':
        arg_file = "run_humanoid3d_backflip_args.txt"
    if (arg_file != ''):
        path = pybullet_data.getDataPath() + "/args/" + arg_file
        if os.path.isfile(path):
            succ = arg_parser.load_file(path)
        else:
            files = [arg_parser.load_file(f) for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]
            succ = all(files)
        Logger.print2(arg_file)
        assert succ, Logger.print2('Failed to load args from: ' + arg_file)
    return arg_parser


args = sys.argv[1:]


def build_world(args, enable_draw):
    arg_parser = build_arg_parser(args)
    print("enable_draw=", enable_draw)
    env = PyBulletDeepMimicEnvMultiClip(arg_parser, enable_draw)
    world = RLWorld(env, arg_parser)

    motion_file = arg_parser.parse_string("motion_file")
    print("motion_file=", motion_file)
    bodies = arg_parser.parse_ints("fall_contact_bodies")
    print("bodies=", bodies)
    int_output_path = arg_parser.parse_string("int_output_path")
    print("int_output_path=", int_output_path)
    agent_files = pybullet_data.getDataPath() + "/" + arg_parser.parse_string("agent_files")

    AGENT_TYPE_KEY = "AgentType"

    print("agent_file=", agent_files)
    with open(agent_files) as data_file:
        json_data = json.load(data_file)
        print("json_data=", json_data)
        assert AGENT_TYPE_KEY in json_data
        agent_type = json_data[AGENT_TYPE_KEY]
        print("agent_type=", agent_type)
        agent = PPOAgent(world, id, json_data)

        agent.set_enable_training(False)
        world.reset()
    return world


if __name__ == '__main__':

    world = build_world(args, True)
    while (world.env._pybullet_client.isConnected()):

        timeStep = update_timestep
        time.sleep(timeStep)
        keys = world.env.getKeyboardEvents()

        if world.env.isKeyTriggered(keys, ' '):
            animating = not animating
        if world.env.isKeyTriggered(keys, 'i'):
            step = True
        if (animating or step):
            update_world(world, timeStep)
            step = False
