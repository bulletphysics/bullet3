# Copyright (c) 2018-present, Facebook, Inc.
# All rights reserved.
#
# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.
#

import numpy as np
import copy
from common.skeleton import Skeleton
from common.mocap_dataset import MocapDataset
from common.camera import normalize_screen_coordinates, image_coordinates
       
humaneva_skeleton = Skeleton(parents=[-1, 0, 1, 2, 3, 1, 5, 6, 0, 8, 9, 0, 11, 12, 1],
       joints_left=[2, 3, 4, 8, 9, 10],
       joints_right=[5, 6, 7, 11, 12, 13])

humaneva_cameras_intrinsic_params = [
    {
        'id': 'C1',
        'res_w': 640,
        'res_h': 480,
        'azimuth': 0, # Only used for visualization
    },
    {
        'id': 'C2',
        'res_w': 640,
        'res_h': 480,
        'azimuth': -90, # Only used for visualization
    },
    {
        'id': 'C3',
        'res_w': 640,
        'res_h': 480,
        'azimuth': 90, # Only used for visualization
    },
]

humaneva_cameras_extrinsic_params = {
    'S1': [
        {
            'orientation': [0.424207, -0.4983646, -0.5802981, 0.4847012],
            'translation': [4062.227,  663.2477, 1528.397],
        },
        {
            'orientation': [0.6503354, -0.7481602, -0.0919284, 0.0941766],
            'translation': [844.8131, -3805.2092,  1504.9929],
        },
        {
            'orientation': [0.0664734, -0.0690535, 0.7416416, -0.6639132],
            'translation': [-797.67377, 3916.3174, 1433.6602],
        },
    ],
    'S2': [
        {
            'orientation': [ 0.4214752, -0.4961493, -0.5838273, 0.4851187 ],
            'translation': [ 4112.9121,   626.4929,  1545.2988], 
        },
        {
            'orientation': [ 0.6501393, -0.7476588, -0.0954617, 0.0959808 ],
            'translation': [  923.5740, -3877.9243,  1504.5518], 
        },
        {
            'orientation': [ 0.0699353, -0.0712403, 0.7421637, -0.662742 ],
            'translation': [ -781.4915,  3838.8853,  1444.9929], 
        },
    ],
    'S3': [
        {
            'orientation': [ 0.424207, -0.4983646, -0.5802981, 0.4847012 ],
            'translation': [ 4062.2271,   663.2477,  1528.3970], 
        },
        {
            'orientation': [ 0.6503354, -0.7481602, -0.0919284, 0.0941766 ],
            'translation': [  844.8131, -3805.2092,  1504.9929], 
        },
        {
            'orientation': [ 0.0664734, -0.0690535, 0.7416416, -0.6639132 ],
            'translation': [ -797.6738,  3916.3174,  1433.6602], 
        },
    ],
    'S4': [
        {},
        {},
        {},
    ],
    
}

class HumanEvaDataset(MocapDataset):
    def __init__(self, path):
        super().__init__(fps=60, skeleton=humaneva_skeleton)
        
        self._cameras = copy.deepcopy(humaneva_cameras_extrinsic_params)
        for cameras in self._cameras.values():
            for i, cam in enumerate(cameras):
                cam.update(humaneva_cameras_intrinsic_params[i])
                for k, v in cam.items():
                    if k not in ['id', 'res_w', 'res_h']:
                        cam[k] = np.array(v, dtype='float32')
                if 'translation' in cam:
                    cam['translation'] = cam['translation']/1000 # mm to meters
                
        for subject in list(self._cameras.keys()):
            data = self._cameras[subject]
            del self._cameras[subject]
            for prefix in ['Train/', 'Validate/', 'Unlabeled/Train/', 'Unlabeled/Validate/', 'Unlabeled/']:
                self._cameras[prefix + subject] = data
        
        # Load serialized dataset
        data = np.load(path)['positions_3d'].item()
        
        self._data = {}
        for subject, actions in data.items():
            self._data[subject] = {}
            for action_name, positions in actions.items():
                self._data[subject][action_name] = {
                    'positions': positions,
                    'cameras': self._cameras[subject],
                }
   