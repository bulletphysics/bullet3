import json
import math
import random
import numpy as np
from os import listdir
from os.path import isfile, join

from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


class MotionCaptureDataMultiClip(object):

    def __init__(self):
        self.Reset()

    def Reset(self):
        self._motion_data = {}
        self._num_frames_min = 9999
        self._num_frames_max = -1
        self._num_clips = -1
        self._names = []

    def Load(self, path):
        if isfile(path):
            files = [path]
        else:
            files = [join(path, f) for f in listdir(path) if isfile(join(path, f))]
            files.sort()
        for i in range(len(files)):
            with open(files[i], 'r') as f:
                self._names.append(files[i].split('/')[-1])
                self._motion_data[i] = json.load(f)
                t = len(self._motion_data[i]['Frames'])
                self._num_frames_min = min(t, self._num_frames_min)
                self._num_frames_max = max(t, self._num_frames_max)
        self._num_clips = len(self._motion_data)
        self._downsample = True
        if self._downsample:
            self.downsampleClips()
            self._num_frames_max = self._num_frames_min
        else:
            self.upsampleClips()
            self._num_frames_min = self._num_frames_max

    def getNumFrames(self):
        if self._downsample:
            return self._num_frames_min
        else:
            return self._num_frames_max

    def getKeyFrameDuration(self, id=0):
        return self._motion_data[id]['Frames'][0][0]

    def getCycleTime(self):
        keyFrameDuration = self.getKeyFrameDuration()
        cycleTime = keyFrameDuration * (self.getNumFrames() - 1)
        return cycleTime

    def calcCycleCount(self, simTime, cycleTime):
        phases = simTime / cycleTime
        count = math.floor(phases)
        return count

    def computeCycleOffset(self, id=0):
        lastFrame = self.getNumFrames() - 1
        frameData = self._motion_data[id]['Frames'][0]
        frameDataNext = self._motion_data[id]['Frames'][lastFrame]

        basePosStart = [frameData[1], frameData[2], frameData[3]]
        basePosEnd = [frameDataNext[1], frameDataNext[2], frameDataNext[3]]
        self._cycleOffset = [
            basePosEnd[0] - basePosStart[0], basePosEnd[1] - basePosStart[1],
            basePosEnd[2] - basePosStart[2]
        ]
        return self._cycleOffset

    def getNumClips(self):
        return self._num_clips

    def downsampleClips(self):
        for i in range(self._num_clips):
            n_frames = len(self._motion_data[i]['Frames'])
            if n_frames != self._num_frames_min:
                sample = random.sample(range(n_frames), self._num_frames_min)
                sample.sort()
                downsampled = np.array(self._motion_data[i]['Frames'])[sample]
                self._motion_data[i]['Frames'] = downsampled.tolist()
                #s = json.dumps(self._motion_data[i])
                #with open("output/{}".format(self._names[i]), 'w') as f:
                #    f.writelines(s)

    def upsampleClips(self):
        print("Max number of frames: ", self._num_frames_max)
        for i in range(self._num_clips):
            #print("Uspsampling clip number: ", i)
            keyframe_duration = self.getKeyFrameDuration(i)
            old_times = np.arange(0, len(self._motion_data[i]['Frames']) * keyframe_duration, keyframe_duration)
            while len(old_times) < self._num_frames_max:
                new_times, new_vals = self.slerpSingleClip(self._motion_data[i]['Frames'], old_times)
                #print("Number of final frames: ", len(new_vals))
                self._motion_data[i]['Frames'] = new_vals
                old_times = new_times
            #s = json.dumps(self._motion_data[i])
            #with open("output/{}".format(self._names[i]), 'w') as f:
            #    f.writelines(s)


    def slerpSingleClip(self, clip, key_times):
        #print("Number of initial frames: ", len(key_times))
        org_clip = self.quatlist_to_quatlists(clip)
        org_clip = np.asarray(org_clip)
        t = org_clip[:, 0]
        root_pos = org_clip[:, 1]
        key_rots = org_clip[:, 2]
        n_frames = len(key_rots)
        assert len(key_times) == n_frames
        needed_frames = self._num_frames_max - n_frames
        #print("Needed frames: ", needed_frames)
        inter_times = self.calc_inter_times(key_times)
        inter_times = sorted(random.sample(inter_times, min(len(inter_times), needed_frames)))
        #print("Number of frames to interpolate: ", len(inter_times))
        #print("Number of rots: ", len(key_rots[0]))
        inter_joint = []
        for i in range(len(key_rots[0])):
            quats = [rot[i] for rot in key_rots]
            if len(quats[0]) == 4:
                joint = R.from_quat(quats)
                slerp = Slerp(key_times, joint)
                interp_rots = slerp(inter_times)
                interp_rots = interp_rots.as_quat().tolist()
            else:
                interp_rots = []
                for tim in range(len(inter_times)):
                    lb = key_times.tolist().index(max([st for st in key_times if st < inter_times[tim]]))
                    ub = lb + 1
                    #print(lb, ub)
                    new_rot = (quats[ub][0] + quats[lb][0])/2
                    interp_rots.append([new_rot])
            inter_joint.append(interp_rots)
        inter_joint = np.array(inter_joint).T
        #print("Shape of interpolated joints: ", inter_joint.shape)
        old_dict = dict(zip(key_times, key_rots))
        new_dict = dict(zip(inter_times, inter_joint))
        old_root_pos = dict(zip(key_times, root_pos))
        inter_root_pos = self.calc_inter_root_pos(root_pos, key_times, inter_times)
        new_root_pos = dict(zip(inter_times, inter_root_pos))
        new_dict = {**old_dict, **new_dict}
        new_rp_dict = {**old_root_pos, **new_root_pos}
        ord_keys = sorted(new_dict.keys())
        ord_rots = [new_dict[k] for k in ord_keys]
        ord_root_pos = [new_rp_dict[k] for k in ord_keys]
        new_clip = self.quatlists_to_quatlist(t, ord_root_pos, ord_rots)

        return np.array(ord_keys), new_clip

    def quatlist_to_quatlists(self, clip):
        new_clips = []
        for c in clip:
            t = c[0]
            root_pos = c[1:4]
            root_rotation = c[4:8]
            chest_rotation = c[8:12]
            neck_rotation = c[12:16]
            right_hip_rotation = c[16:20]
            right_knee_rotation = c[20]
            right_ankle_rotation = c[21:25]
            right_shoulder_rotation = c[25:29]
            right_elbow_rotation = c[29]
            left_hip_rotation = c[30:34]
            left_knee_rotation = c[34]
            left_ankle_rotation = c[35:39]
            left_shoulder_rotation = c[39:43]
            left_elbow_rotation = c[43]
            d = [
                t,
                root_pos,
                [
                    self.deepmimic_to_scipy_quaternion(root_rotation),
                    self.deepmimic_to_scipy_quaternion(chest_rotation),
                    self.deepmimic_to_scipy_quaternion(neck_rotation),
                    self.deepmimic_to_scipy_quaternion(right_hip_rotation),
                    [right_knee_rotation],
                    self.deepmimic_to_scipy_quaternion(right_ankle_rotation),
                    self.deepmimic_to_scipy_quaternion(right_shoulder_rotation),
                    [right_elbow_rotation],
                    self.deepmimic_to_scipy_quaternion(left_hip_rotation),
                    [left_knee_rotation],
                    self.deepmimic_to_scipy_quaternion(left_ankle_rotation),
                    self.deepmimic_to_scipy_quaternion(left_shoulder_rotation),
                    [left_elbow_rotation]
                ]
            ]
            new_clips.append(d)
        return new_clips

    def deepmimic_to_scipy_quaternion(self, quat):
        return quat[1:] + [quat[0]]

    def scipy_to_deepmimic_quaternion(self, quat):
        return [quat[-1]] + quat[:-1]

    def calc_inter_times(self, times, method="intermediate"):
        if method == "intermediate":
            inter_times = []
            for i in range(1, len(times)):
                it = (times[i] - times[i-1])/2 + times[i-1]
                inter_times.append(it)
            return inter_times

    def calc_inter_root_pos(self, root_pos, times, inter_times):
        inter_root_pos = []
        all_times = sorted([*times.tolist(), *inter_times])
        for i in range(len(inter_times)):
            low_index = times.tolist().index(all_times[all_times.index(inter_times[i]) - 1])
            up_index = times.tolist().index(all_times[all_times.index(inter_times[i]) + 1])
            assert low_index == up_index - 1
            inter_root_pos.append(((np.array(root_pos[up_index]) + np.array(root_pos[low_index]))/2).tolist())
        return inter_root_pos

    def quatlists_to_quatlist(self, t, ord_root_pos, ord_rots):
        delta_t = t[0]
        new_quats = self.merge_quaternions(ord_rots)
        a = []
        for i in range(len(ord_root_pos)):
            rot = new_quats[i]

            a.append([
                delta_t,
                *ord_root_pos[i],
                *rot
            ])
        return a

    def merge_quaternions(self, rotations):
        quats = []
        for rot in rotations:
            rots = []
            for r in rot:
                if len(r) == 4:
                    r = self.scipy_to_deepmimic_quaternion(r)
                rots += [el for el in r]
            quats.append(rots)
        return quats