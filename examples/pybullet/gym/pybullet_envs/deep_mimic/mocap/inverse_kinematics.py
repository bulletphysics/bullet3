# coding: utf-8

import numpy as np
from math import *
import pickle
import json
# import quaternion
from transformation import *

from pyquaternion import Quaternion

def get_angle(vec1, vec2):
    cos_theta = np.dot(vec1, vec2)/(np.linalg.norm(vec1) * np.linalg.norm(vec2))
    return acos(cos_theta) 


def get_quaternion(ox, oy, oz, x, y, z):
    # given transformed axis in x-y-z order return a quaternion
    ox /= np.linalg.norm(ox)
    oy /= np.linalg.norm(oy)
    oz /= np.linalg.norm(oz)

    set1 = np.vstack((ox,oy,oz))

    x /= np.linalg.norm(x)
    y /= np.linalg.norm(y)
    z /= np.linalg.norm(z)

    set2 = np.vstack((x,y,z))
    rot_mat = superimposition_matrix(set1, set2, scale=False, usesvd=True)
    rot_qua = quaternion_from_matrix(rot_mat)

    return rot_qua


# 3D coord to deepmimic rotations
def coord_to_rot(frameNum, frame, frame_duration):
    eps = 0.001
    axis_rotate_rate = 0.3

    frame = np.array(frame)
    tmp = [[] for i in range(15)]
    # duration of frame in seconds (1D),
    tmp[0] = [frame_duration]
    # root position (3D),
    tmp[1] = frame[0]
    # root rotation (4D),
    root_y = (frame[7] - frame[0])
    root_z = (frame[1] - frame[0]) 
    root_x = np.cross(root_y, root_z)

    x = np.array([1.0,0,0])
    y = np.array([0,1.0,0])
    z = np.array([0,0,1.0])
    
    rot_qua = get_quaternion(root_x, root_y, root_z, x, y, z)
    tmp[2] = list(rot_qua)

    # chest rotation (4D),
    chest_y = (frame[8] - frame[7])
    chest_z = (frame[14] - frame[8]) 
    chest_x = np.cross(chest_y, chest_z)
    rot_qua = get_quaternion(chest_x, chest_y, chest_z, root_x, root_y, root_z)
    tmp[3] = list(rot_qua)

    # neck rotation (4D),
    neck_y = (frame[10] - frame[8])
    neck_z = np.cross(frame[10]-frame[9], frame[8]-frame[9]) 
    neck_x = np.cross(neck_y, neck_z)
    rot_qua = get_quaternion(neck_x, neck_y, neck_z, chest_x, chest_y, chest_z)
    tmp[4] = list(rot_qua)

    # right hip rotation (4D),
    r_hip_y = (frame[1] - frame[2])
    r_hip_z = np.cross(frame[1]-frame[2], frame[3]-frame[2]) 
    r_hip_x = np.cross(r_hip_y, r_hip_z)
    rot_qua = get_quaternion(r_hip_x, r_hip_y, r_hip_z, root_x, root_y, root_z)
    tmp[5] = list(rot_qua)

    # right knee rotation (1D),
    vec1 = frame[1] - frame[2]
    vec2 = frame[3] - frame[2]
    angle1 = get_angle(vec1, vec2)
    tmp[6] = [angle1-pi]

    # right ankle rotation (4D),
    tmp[7] = [1,0,0,0]

    #  right shoulder rotation (4D),
    r_shou_y = (frame[14] - frame[15])
    r_shou_z = np.cross(frame[16]-frame[15], frame[14]-frame[15]) 
    r_shou_x = np.cross(r_shou_y, r_shou_z)
    rot_qua = get_quaternion(r_shou_x, r_shou_y, r_shou_z, chest_x, chest_y, chest_z)
    tmp[8] = list(rot_qua)

    # right elbow rotation (1D),
    vec1 = frame[14] - frame[15]
    vec2 = frame[16] - frame[15]
    angle1 = get_angle(vec1, vec2)
    tmp[9] = [pi-angle1]

    # left hip rotation (4D),
    l_hip_y = (frame[4] - frame[5])
    l_hip_z = np.cross(frame[4]-frame[5], frame[6]-frame[5]) 
    l_hip_x = np.cross(l_hip_y, l_hip_z)
    rot_qua = get_quaternion(l_hip_x, l_hip_y, l_hip_z, root_x, root_y, root_z)    
    tmp[10] = list(rot_qua)
    
    # left knee rotation (1D),
    vec1 = frame[4] - frame[5]
    vec2 = frame[6] - frame[5]
    angle1 = get_angle(vec1, vec2)
    tmp[11] = [angle1-pi]
    
    # left ankle rotation (4D),
    tmp[12] = [1,0,0,0]

    # left shoulder rotation (4D),
    l_shou_y = (frame[11] - frame[12])
    l_shou_z = np.cross(frame[13]-frame[12], frame[11]-frame[12]) 
    l_shou_x = np.cross(l_shou_y, l_shou_z)
    rot_qua = get_quaternion(l_shou_x, l_shou_y, l_shou_z, chest_x, chest_y, chest_z)
    tmp[13] = list(rot_qua)

    # left elbow rotation (1D)
    vec1 = frame[11] - frame[12]
    vec2 = frame[13] - frame[12]
    angle1 = get_angle(vec1, vec2)
    tmp[14] = [pi-angle1]

    ret = []
    for i in tmp:
        ret += list(i)
    return np.array(ret)

# In[6]:


def coord_seq_to_rot_seq(coord_seq, frame_duration):
    ret = []
    for i in range(len(coord_seq)):
        tmp = coord_to_rot( i, coord_seq[i], frame_duration)
        ret.append(list(tmp))
    return ret






