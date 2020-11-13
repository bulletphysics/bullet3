import numpy as np
import math
from pybullet_envs.deep_mimic.env.env import Env
from pybullet_envs.deep_mimic.env.action_space import ActionSpace
from pybullet_utils import bullet_client
import time
from pybullet_envs.deep_mimic.env import motion_capture_data_multiclip
from pybullet_envs.deep_mimic.env import humanoid_stable_pd_multiclip
import pybullet_data
import pybullet as p1
import random

from enum import Enum


class InitializationStrategy(Enum):
    """Set how the environment is initialized."""
    START = 0
    RANDOM = 1  # random state initialization (RSI)


class PyBulletDeepMimicEnvMultiClip(Env):

    def __init__(self, arg_parser=None, enable_draw=False, pybullet_client=None,
                 time_step=1. / 240,
                 init_strategy=InitializationStrategy.RANDOM):
        super().__init__(arg_parser, enable_draw)
        self._num_agents = 1
        self._pybullet_client = pybullet_client
        self._isInitialized = False
        #self._useStablePD = True
        self._arg_parser = arg_parser
        self.timeStep = time_step
        self._init_strategy = init_strategy
        self._n_clips = -1
        print("Initialization strategy: {:s}".format(init_strategy))
        self.reset()

    def reset(self):

        if not self._isInitialized:
            if self.enable_draw:
                self._pybullet_client = bullet_client.BulletClient(connection_mode=p1.GUI)
                # disable 'GUI' since it slows down a lot on Mac OSX and some other platforms
                self._pybullet_client.configureDebugVisualizer(self._pybullet_client.COV_ENABLE_GUI, 0)
            else:
                self._pybullet_client = bullet_client.BulletClient()

            self._pybullet_client.setAdditionalSearchPath(pybullet_data.getDataPath())
            z2y = self._pybullet_client.getQuaternionFromEuler([-math.pi * 0.5, 0, 0])
            self._planeId = self._pybullet_client.loadURDF("plane_implicit.urdf", [0, 0, 0],
                                                           z2y,
                                                           useMaximalCoordinates=True)
            # print("planeId=",self._planeId)
            self._pybullet_client.configureDebugVisualizer(self._pybullet_client.COV_ENABLE_Y_AXIS_UP, 1)
            self._pybullet_client.setGravity(0, -9.8, 0)

            self._pybullet_client.setPhysicsEngineParameter(numSolverIterations=10)
            self._pybullet_client.changeDynamics(self._planeId, linkIndex=-1, lateralFriction=0.9)

            self._mocapData = motion_capture_data_multiclip.MotionCaptureDataMultiClip()

            motion_file = self._arg_parser.parse_strings('motion_file')
            print("motion_file=", motion_file[0])

            motionPath = pybullet_data.getDataPath() + "/" + motion_file[0]

            self._mocapData.Load(motionPath)
            self._n_clips = self._mocapData.getNumClips()
            timeStep = self.timeStep
            useFixedBase = False
            self._humanoid = humanoid_stable_pd_multiclip.HumanoidStablePDMultiClip(self._pybullet_client, self._mocapData,
                                                                                    timeStep, useFixedBase, self._arg_parser)
            self._isInitialized = True

            self._pybullet_client.setTimeStep(timeStep)
            self._pybullet_client.setPhysicsEngineParameter(numSubSteps=1)

        # print("numframes = ", self._humanoid._mocap_data.NumFrames())

        if self._init_strategy == InitializationStrategy.RANDOM:
            rnrange = 1000
            rn = random.randint(0, rnrange)
            startTime = float(rn) / rnrange * self._humanoid.getCycleTime()
        elif self._init_strategy == InitializationStrategy.START:
            startTime = 0

        self.t = startTime

        self._humanoid.setSimTime(startTime)

        self._humanoid.resetPose()
        # this clears the contact points. Todo: add API to explicitly clear all contact points?
        self._humanoid.resetPose()
        self.needs_update_time = self.t - 1  # force update

    def get_num_agents(self):
        return self._num_agents

    def get_action_space(self, agent_id):
        return ActionSpace(ActionSpace.Continuous)

    def get_reward_min(self, agent_id):
        return 0

    def get_reward_max(self, agent_id):
        return 1

    def get_reward_fail(self, agent_id):
        return self.get_reward_min(agent_id)

    def get_reward_succ(self, agent_id):
        return self.get_reward_max(agent_id)

    # scene_name == "imitate" -> cDrawSceneImitate
    def get_state_size(self, agent_id):
        # cCtController::GetStateSize()
        # int state_size = cDeepMimicCharController::GetStateSize();
        #                     state_size += GetStatePoseSize();#106
        #                     state_size += GetStateVelSize(); #(3+3)*numBodyParts=90
        # state_size += GetStatePhaseSize();#1
        # 197
        return 197

    def build_state_norm_groups(self, agent_id):
        groups = [0] * self.get_state_size(agent_id)
        groups[0] = -1
        return groups

    def build_state_offset(self, agent_id):
        out_offset = [0] * self.get_state_size(agent_id)
        phase_offset = -0.5
        out_offset[0] = phase_offset
        return np.array(out_offset)

    def build_state_scale(self, agent_id):
        out_scale = [1] * self.get_state_size(agent_id)
        phase_scale = 2
        out_scale[0] = phase_scale
        return np.array(out_scale)

    def get_goal_size(self, agent_id):
        return 0

    def get_action_size(self, agent_id):
        ctrl_size = 43  # numDof
        root_size = 7
        return ctrl_size - root_size

    def build_goal_norm_groups(self, agent_id):
        return np.array([])

    def build_goal_offset(self, agent_id):
        return np.array([])

    def build_goal_scale(self, agent_id):
        return np.array([])

    def build_action_offset(self, agent_id):
        out_offset = [
            0.0000000000, 0.0000000000, 0.0000000000, -0.200000000, 0.0000000000, 0.0000000000,
            0.0000000000, -0.200000000, 0.0000000000, 0.0000000000, 0.00000000, -0.2000000, 1.57000000,
            0.00000000, 0.00000000, 0.00000000, -0.2000000, 0.00000000, 0.00000000, 0.00000000,
            -0.2000000, -1.5700000, 0.00000000, 0.00000000, 0.00000000, -0.2000000, 1.57000000,
            0.00000000, 0.00000000, 0.00000000, -0.2000000, 0.00000000, 0.00000000, 0.00000000,
            -0.2000000, -1.5700000
        ]
        # see cCtCtrlUtil::BuildOffsetScalePDPrismatic and
        # see cCtCtrlUtil::BuildOffsetScalePDSpherical
        return np.array(out_offset)

    def build_action_scale(self, agent_id):
        # see cCtCtrlUtil::BuildOffsetScalePDPrismatic and
        # see cCtCtrlUtil::BuildOffsetScalePDSpherical
        out_scale = [
            0.20833333333333, 1.00000000000000, 1.00000000000000, 1.00000000000000, 0.25000000000000,
            1.00000000000000, 1.00000000000000, 1.00000000000000, 0.12077294685990, 1.00000000000000,
            1.000000000000, 1.000000000000, 0.159235668789, 0.159235668789, 1.000000000000,
            1.000000000000, 1.000000000000, 0.079617834394, 1.000000000000, 1.000000000000,
            1.000000000000, 0.159235668789, 0.120772946859, 1.000000000000, 1.000000000000,
            1.000000000000, 0.159235668789, 0.159235668789, 1.000000000000, 1.000000000000,
            1.000000000000, 0.107758620689, 1.000000000000, 1.000000000000, 1.000000000000,
            0.159235668789
        ]
        return np.array(out_scale)

    def build_action_bound_min(self, agent_id):
        # see cCtCtrlUtil::BuildBoundsPDSpherical
        out_scale = [
            -4.79999999999, -1.00000000000, -1.00000000000, -1.00000000000, -4.00000000000,
            -1.00000000000, -1.00000000000, -1.00000000000, -7.77999999999, -1.00000000000,
            -1.000000000, -1.000000000, -7.850000000, -6.280000000, -1.000000000, -1.000000000,
            -1.000000000, -12.56000000, -1.000000000, -1.000000000, -1.000000000, -4.710000000,
            -7.779999999, -1.000000000, -1.000000000, -1.000000000, -7.850000000, -6.280000000,
            -1.000000000, -1.000000000, -1.000000000, -8.460000000, -1.000000000, -1.000000000,
            -1.000000000, -4.710000000
        ]

        return out_scale

    def build_action_bound_max(self, agent_id):
        out_scale = [
            4.799999999, 1.000000000, 1.000000000, 1.000000000, 4.000000000, 1.000000000, 1.000000000,
            1.000000000, 8.779999999, 1.000000000, 1.0000000, 1.0000000, 4.7100000, 6.2800000,
            1.0000000, 1.0000000, 1.0000000, 12.560000, 1.0000000, 1.0000000, 1.0000000, 7.8500000,
            8.7799999, 1.0000000, 1.0000000, 1.0000000, 4.7100000, 6.2800000, 1.0000000, 1.0000000,
            1.0000000, 10.100000, 1.0000000, 1.0000000, 1.0000000, 7.8500000
        ]
        return out_scale

    def set_mode(self, mode):
        self._mode = mode

    def need_new_action(self, agent_id):
        if self.t >= self.needs_update_time:
            self.needs_update_time = self.t + 1. / 30.
            return True
        return False

    def record_state(self, agent_id):
        state = self._humanoid.getState()
        return np.array(state)

    def record_goal(self, agent_id):
        return np.array([])

    def calc_reward(self, agent_id):
        rewards = {}
        for i in range(self._n_clips):
            kinPose = self._humanoid.computePose(self._humanoid._frameFraction, i)
            rewards[i] = self._humanoid.getReward(kinPose, i)
        reward = self.select_reward(rewards)
        return reward

    def set_action(self, agent_id, action):
        # print("action=",)
        # for a in action:
        #  print(a)
        self.desiredPose = {}
        for i in range(self._n_clips):
            self.desiredPose[i] = self._humanoid.convertActionToPose(action, i)
            # we need the target root positon and orientation to be zero, to be compatible with deep mimic
            self.desiredPose[i][0] = 0
            self.desiredPose[i][1] = 0
            self.desiredPose[i][2] = 0
            self.desiredPose[i][3] = 0
            self.desiredPose[i][4] = 0
            self.desiredPose[i][5] = 0
            self.desiredPose[i][6] = 0
            target_pose = np.array(self.desiredPose[i])

        # print("set_action: desiredPose=", self.desiredPose)

    def log_val(self, agent_id, val):
        pass

    def update(self, timeStep):
        # print("pybullet_deep_mimic_env:update timeStep=",timeStep," t=",self.t)
        self._pybullet_client.setTimeStep(timeStep)
        self._humanoid._timeStep = timeStep
        self.timeStep = timeStep

        self.t += timeStep
        self._humanoid.setSimTime(self.t)

        if self.desiredPose:
            for i in range(self._n_clips):
                kinPose = self._humanoid.computePose(self._humanoid._frameFraction, i)
                self._humanoid.initializePose(self._humanoid._poseInterpolators[i],
                                              self._humanoid._kin_models[i],
                                              initBase=True)
            # print("desiredPositions=",self.desiredPose[i])
            maxForces = [
                0, 0, 0, 0, 0, 0, 0, 200, 200, 200, 200, 50, 50, 50, 50, 200, 200, 200, 200, 150, 90,
                90, 90, 90, 100, 100, 100, 100, 60, 200, 200, 200, 200, 150, 90, 90, 90, 90, 100, 100,
                100, 100, 60
            ]

            self._humanoid.computeAndApplyPDForces(self.desiredPose[0], maxForces=maxForces) # TODO check if this should be inside the for loop

            self._pybullet_client.stepSimulation()

    def set_sample_count(self, count):
        return

    def check_terminate(self, agent_id):
        return Env.Terminate(self.is_episode_end())

    def is_episode_end(self):
        isEnded = self._humanoid.terminates()
        # also check maximum time, 20 seconds (todo get from file)
        # print("self.t=",self.t)
        if (self.t > 20):
            isEnded = True
        return isEnded

    def check_valid_episode(self):
        # could check if limbs exceed velocity threshold
        return True

    def getKeyboardEvents(self):
        return self._pybullet_client.getKeyboardEvents()

    def isKeyTriggered(self, keys, key):
        o = ord(key)
        # print("ord=",o)
        if o in keys:
            return keys[ord(key)] & self._pybullet_client.KEY_WAS_TRIGGERED
        return False

    def select_reward(self, rewards, criterion=None):
        # todo create enum for criterion
        return max(rewards.values())
