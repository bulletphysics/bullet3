import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

print('parent:', parentdir)

import pybullet_data
import pybullet
import time
import random

from pybullet_utils.bullet_client import BulletClient
from deep_mimic.env.motion_capture_data import MotionCaptureData

# from pybullet_envs.deep_mimic.env.humanoid_stable_pd import HumanoidStablePd
from humanoid import Humanoid
from humanoid import HumanoidPose

# from env.humanoid_stable_pd

from deepmimic_json_generator import *

import pybullet as p
import numpy as np
import argparse

parser = argparse.ArgumentParser(description='Arguments for loading reference for learning.')

# General arguments
parser.add_argument('--dataset_path',
                    default='data/data_3d_h36m.npz',
                    type=str,
                    help='target dataset')  # h36m or humaneva
parser.add_argument(
    '--json_path',
    default='data/Walking.json',
    type=str,
    help='json file path for storing the deepmimic-format json created by inverse-kinect.')
parser.add_argument('--fps', default=24, type=int, help='frame per second')
parser.add_argument('--subject', default='S11', type=str, help='camera subject.')
parser.add_argument('--action', default='Walking', type=str, help='name of the action.')
parser.add_argument('--loop',
                    default='wrap',
                    type=str,
                    help='loop information in deepmimic, wrap or none.')
parser.add_argument('--draw_gt', action='store_true', help='draw ground truth or not.')

args = parser.parse_args()

dataset_path = args.dataset_path
json_path = args.json_path
fps = args.fps
subject = args.subject
action = args.action
loop = args.loop
draw_gt = args.draw_gt


def draw_ground_truth(coord_seq, frame, duration, shift):
  global joint_info
  joint = coord_seq[frame]
  shift = np.array(shift)
  for i in range(1, 17):
    # print(x[11], x[14])
    joint_fa = joint_info['father'][i]
    if joint_info['side'][i] == 'right':
      p.addUserDebugLine(lineFromXYZ=joint[i] + shift,
                         lineToXYZ=joint[joint_fa] + shift,
                         lineColorRGB=(255, 0, 0),
                         lineWidth=1,
                         lifeTime=duration)
    else:
      p.addUserDebugLine(lineFromXYZ=joint[i] + shift,
                         lineToXYZ=joint[joint_fa] + shift,
                         lineColorRGB=(0, 0, 0),
                         lineWidth=1,
                         lifeTime=duration)


dataset = init_fb_h36m_dataset(dataset_path)
ground_truth = pose3D_from_fb_h36m(dataset, subject=subject, action=action, shift=[1.0, 0.0, 0.0])

rot_seq = coord_seq_to_rot_seq(coord_seq=ground_truth, frame_duration=1 / fps)

rot_seq_to_deepmimic_json(rot_seq=rot_seq, loop=loop, json_path=json_path)

bc = BulletClient(connection_mode=pybullet.GUI)
bc.setAdditionalSearchPath(pybullet_data.getDataPath())
bc.configureDebugVisualizer(bc.COV_ENABLE_Y_AXIS_UP, 1)
bc.setGravity(0, -9.8, 0)
motion = MotionCaptureData()

motionPath = json_path
motion.Load(motionPath)
print("numFrames = ", motion.NumFrames())

simTimeId = bc.addUserDebugParameter("simTime", 0, motion.NumFrames() - 1.1, 0)

y2zOrn = bc.getQuaternionFromEuler([-1.57, 0, 0])
bc.loadURDF("plane.urdf", [0, -0.04, 0], y2zOrn)

humanoid = Humanoid(bc, motion, [0, 0, 0])  #这是初始位置的坐标

print(p.getBasePositionAndOrientation(humanoid._humanoid))

simTime = 0
keyFrameDuration = motion.KeyFrameDuraction()
print("keyFrameDuration=", keyFrameDuration)
for utNum in range(motion.NumFrames()):
  bc.stepSimulation()
  humanoid.RenderReference(utNum * keyFrameDuration)
  if draw_gt:
    draw_ground_truth(coord_seq=ground_truth,
                      frame=utNum,
                      duration=keyFrameDuration,
                      shift=[-1.0, 0.0, 1.0])
  time.sleep(0.001)
stage = 0


def Reset(humanoid):
  global simTime
  humanoid.Reset()
  simTime = 0
  humanoid.SetSimTime(simTime)
  pose = humanoid.InitializePoseFromMotionData()
  humanoid.ApplyPose(pose, True, True, humanoid._humanoid, bc)


Reset(humanoid)
p.disconnect()
