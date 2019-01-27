import argparse
from pybullet_envs.minitaur.envs import minitaur_logging

parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('--log_file', help='path to protobuf file', default='')
args = parser.parse_args()
logging = minitaur_logging.MinitaurLogging()
episode = logging.restore_episode(args.log_file)
print("episode=",episode)