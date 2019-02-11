from inverse_kinematics import *
from h36m_dataset import Human36mDataset
from camera import *

import numpy as np
# In[2]: 

joint_info = {
	'joint_name':['root', 'right_hip', 'right_knee', 'right_ankle', 'left_hip', 'left_knee', 'left_ankle', 'chest', 'neck', 'nose', 'eye', 'left_shoulder', 'left_elbow', 'left_wrist', 'right_shoulder', 'right_elbow', 'right_wrist'],
    'father':[0, 0, 1, 2, 0, 4, 5, 0, 7, 8, 9, 8, 11, 12, 8, 14, 15],
    'side':['middle', 'right', 'right', 'right', 'left', 'left', 'left', 'middle', 'middle', 'middle', 'middle', 'left', 'left', 'left', 'right', 'right', 'right'] 
}

# In[3]:

def init_fb_h36m_dataset(dataset_path):
    dataset = Human36mDataset(dataset_path)
    print('Preparing Facebook Human3.6M Dataset...')
    for subject in dataset.subjects():
        for action in dataset[subject].keys():
            anim = dataset[subject][action]
            positions_3d = []
            for cam in anim['cameras']:
                pos_3d = world_to_camera(anim['positions'], R=cam['orientation'], t=cam['translation'])
                pos_3d[:, 1:] -= pos_3d[:, :1] # Remove global offset, but keep trajectory in first position
                positions_3d.append(pos_3d)
            anim['positions_3d'] = positions_3d
    return dataset

def pose3D_from_fb_h36m(dataset, subject, action, shift):
    pose_seq = dataset[subject][action]['positions_3d'][0].copy()
    trajectory = pose_seq[:, :1]
    pose_seq[:, 1:] += trajectory
    # Invert camera transformation
    cam = dataset.cameras()[subject][0]
    pose_seq = camera_to_world(pose_seq, 
                                   R=cam['orientation'], 
                                   t=cam['translation'])
    x = pose_seq[:,:,0:1]
    y = pose_seq[:,:,1:2] * -1
    z = pose_seq[:,:,2:3] 
    pose_seq = np.concatenate((x,z,y),axis=2)
    # plus shift
    pose_seq += np.array([[shift for i in range(pose_seq.shape[1])] for j in range(pose_seq.shape[0])])
    return pose_seq

def rot_seq_to_deepmimic_json(rot_seq, loop, json_path):
    to_json = {"Loop": loop, "Frames":[]}
    rot_seq = np.around(rot_seq, decimals=6)
    to_json["Frames"] = rot_seq.tolist()
    # In[14]:
    to_file = json.dumps(to_json)
    file = open(json_path,"w")
    file.write(to_file)
    file.close()

