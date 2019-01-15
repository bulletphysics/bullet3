
# coding: utf-8

import os,  inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)

from pybullet_utils.bullet_client import BulletClient
from pybullet_envs.deep_mimic.motion_capture_data import MotionCaptureData
# import pybullet_data
import pybullet
import time
import random

from humanoid import Humanoid
from humanoid import HumanoidPose

import pybullet as p
import numpy as np


from deepmimic_json_generator import *


def draw_ground_truth(coord_seq, frame, duration, shift):
    global joint_info
    joint = coord_seq[frame]
    shift = np.array(shift)
    for i in range(1, 17):
        # print(x[11], x[14])
        joint_fa = joint_info['father'][i]
        if joint_info['side'][i] == 'right':
            p.addUserDebugLine(lineFromXYZ=joint[i]+shift,
                               lineToXYZ=joint[joint_fa]+shift,
                               lineColorRGB=(255,0,0),
                               lineWidth=1,
                               lifeTime=duration)
        else:
            p.addUserDebugLine(lineFromXYZ=joint[i]+shift,
                               lineToXYZ=joint[joint_fa]+shift,
                               lineColorRGB=(0,0,0),
                               lineWidth=1,
                               lifeTime=duration)


# fb_h36m_dataset_path = 'data/data_3d_h36m.npz'
# dataset = init_fb_h36m_dataset(fb_h36m_dataset_path)
# ground_truth = pose3D_from_fb_h36m(dataset, 
#                                    subject = 'S11', 
#                                    action = 'Walking',
#                                    shift = [1.0,0.0,0.0])

# # if load data from facebook output, set y-shift to 0.85
# fb_prediction_dataset_path = 'data/tmp_3d_pose_data.pkl'
# dataset = init_fb_prediction_dataset(fb_prediction_dataset_path)
# ground_truth = pose3D_from_fb_prediction(  dataset, 
#                                            subject = 1, 
#                                            shift = [1.0,0.85,0.0])

# # if load data from facebook output, set y-shift to 0.85
fb_prediction_json_dataset_path = 'data/pose3d-results-0.json'
dataset = init_fb_prediction_json_dataset(fb_prediction_json_dataset_path)
ground_truth = pose3D_from_fb_prediction_json(  dataset, 
                                                shift = [1.0,0.85,0.0])

coord_seq = ground_truth
rot_seq =  coord_seq_to_rot_seq(coord_seq = coord_seq, 
                                frame_duration = 1/24)



# bullet logic

bc = BulletClient(connection_mode=pybullet.GUI)
# bc.setAdditionalSearchPath(os.getcwd())
bc.configureDebugVisualizer(bc.COV_ENABLE_Y_AXIS_UP,1)
bc.setGravity(0,-9.8,0)
motion=MotionCaptureData()

# motionPath = pybullet_data.getDataPath()+"/motions/humanoid3d_backflip.txt"#humanoid3d_spinkick.txt"#/motions/humanoid3d_backflip.txt"
motionPath = 'data/test_dance.json'
motion.Load(motionPath)
print("numFrames = ", motion.NumFrames())


simTimeId= bc.addUserDebugParameter("simTime",0,motion.NumFrames()-1.1,0)

y2zOrn = bc.getQuaternionFromEuler([-1.57,0,0])
bc.loadURDF("data/plane.urdf",[0,-0.04,0], y2zOrn)

# humanoid = Humanoid(bc, motion, [0,0,0])#4000,0,5000])
humanoid = Humanoid(bc, motion, [0,0,0]) #这是初始位置的坐标


print(p.getBasePositionAndOrientation(humanoid._humanoid))

simTime = 0
keyFrameDuration = motion.KeyFrameDuraction()
print("keyFrameDuration=",keyFrameDuration)
for utNum in range(motion.NumFrames()):
    bc.stepSimulation()
    # humanoid.ApplyPose(pose, True, True, humanoid._humanoid,bc)
    humanoid.myApplyPose(utNum * keyFrameDuration)
    draw_ground_truth(coord_seq = coord_seq, 
                      frame = utNum, 
                      duration = keyFrameDuration,
                      shift = [-1.0, 0.0, 1.0])
    time.sleep(0.001)
#     print(utNum, motion._motion_data['Frames'][utNum][4:8])
stage = 0


def Reset(humanoid):
	global simTime
	humanoid.Reset()
	simTime = 0 #random.randint(0,motion.NumFrames()-2)
	humanoid.SetSimTime(simTime)
	pose = humanoid.InitializePoseFromMotionData()
	humanoid.ApplyPose(pose, True, True, humanoid._humanoid,bc)


Reset(humanoid)
p.disconnect()
#bc.stepSimulation()

