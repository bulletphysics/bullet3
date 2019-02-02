import numpy as np
import math
from pybullet_envs.deep_mimic.env.env import Env
from pybullet_envs.deep_mimic.env.action_space import ActionSpace
from pybullet_envs.minitaur.envs import bullet_client
import time
import motion_capture_data
from pybullet_envs.deep_mimic.env import humanoid_stable_pd
import pybullet_data
import pybullet as p1
import random
      
      
class PyBulletDeepMimicEnv(Env):
    def __init__(self, args=None, enable_draw=False, pybullet_client=None):
      super().__init__(args, enable_draw)
      self._num_agents = 1
      self._pybullet_client = pybullet_client
      self._isInitialized = False
      self.reset()
    
    def reset(self):
      self.t = 0
      if not self._isInitialized:
        if self.enable_draw:
          self._pybullet_client =  bullet_client.BulletClient(connection_mode=p1.GUI)
        else:
          self._pybullet_client  =  bullet_client.BulletClient()
        
        self._pybullet_client.setAdditionalSearchPath(pybullet_data.getDataPath())
        z2y = self._pybullet_client.getQuaternionFromEuler([-math.pi*0.5,0,0])
        self._planeId = self._pybullet_client.loadURDF("plane.urdf",[0,0,0],z2y)
        #print("planeId=",self._planeId)
        self._pybullet_client.configureDebugVisualizer(self._pybullet_client.COV_ENABLE_Y_AXIS_UP,1)
        self._pybullet_client.setGravity(0,-9.8,0)
        
        self._pybullet_client.setPhysicsEngineParameter(numSolverIterations=10)
        self._pybullet_client.changeDynamics(self._planeId, linkIndex=-1, lateralFriction=0.9)
        
        self._mocapData = motion_capture_data.MotionCaptureData()
        motionPath = pybullet_data.getDataPath()+"/motions/humanoid3d_walk.txt"
        #motionPath = pybullet_data.getDataPath()+"/motions/humanoid3d_backflip.txt"
        self._mocapData.Load(motionPath)
        timeStep = 1./600
        useFixedBase=False
        self._humanoid = humanoid_stable_pd.HumanoidStablePD(self._pybullet_client, self._mocapData, timeStep, useFixedBase)
        self._isInitialized = True
        
        self._pybullet_client.setTimeStep(timeStep)
        self._pybullet_client.setPhysicsEngineParameter(numSubSteps=2)
        
        
        selfCheck = False
        if (selfCheck):
          curTime = 0
          while self._pybullet_client.isConnected():
            self._humanoid.setSimTime(curTime)
            state = self._humanoid.getState()
            #print("state=",state)
            pose = self._humanoid.computePose(self._humanoid._frameFraction)
            for i in range (10):
              curTime+=timeStep
              #taus = self._humanoid.computePDForces(pose)
              #self._humanoid.applyPDForces(taus)
              #self._pybullet_client.stepSimulation()
            time.sleep(timeStep)
      #print("numframes = ", self._humanoid._mocap_data.NumFrames())
      startTime = random.randint(0,self._humanoid._mocap_data.NumFrames()-2)
      rnrange = 1000
      rn = random.randint(0,rnrange)
      startTime = float(rn)/rnrange * self._humanoid.getCycleTime()

      self._humanoid.setSimTime(startTime)
      self._humanoid.resetPose()
      #this clears the contact points. Todo: add API to explicitly clear all contact points?
      self._pybullet_client.stepSimulation()
      
       	
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
    	
    #scene_name == "imitate" -> cDrawSceneImitate
    def get_state_size(self, agent_id):
        #cCtController::GetStateSize()
        #int state_size = cDeepMimicCharController::GetStateSize();
        #                     state_size += GetStatePoseSize();#106
	      #                     state_size += GetStateVelSize(); #(3+3)*numBodyParts=90
        #state_size += GetStatePhaseSize();#1
        #197
        return 197

    def build_state_norm_groups(self, agent_id):
        #if (mEnablePhaseInput)
	      #{
		    #int phase_group = gNormGroupNone;
		    #int phase_offset = GetStatePhaseOffset();
		    #int phase_size = GetStatePhaseSize();
		    #out_groups.segment(phase_offset, phase_size) = phase_group * Eigen::VectorXi::Ones(phase_size);
        groups = [0]*self.get_state_size(agent_id)
        groups[0] = -1
        return groups	
        
    def build_state_offset(self, agent_id):
        out_offset = [0]*self.get_state_size(agent_id)
        phase_offset = -0.5
        out_offset[0] = phase_offset
        return np.array(out_offset)
    
    def build_state_scale(self, agent_id):
        out_scale = [1]*self.get_state_size(agent_id)
        phase_scale = 2
        out_scale[0] = phase_scale
        return np.array(out_scale)

    def get_goal_size(self, agent_id):
        return 0

    def get_action_size(self, agent_id):
     	  ctrl_size = 43 #numDof
     	  root_size = 7
     	  return ctrl_size - root_size
     	  
    def build_goal_norm_groups(self, agent_id):
        return np.array([])
        
    def build_goal_offset(self, agent_id):
        return np.array([])
    
    def build_goal_scale(self, agent_id):
        return np.array([])
        
    def build_action_offset(self, agent_id):
    	  out_offset = [0] * self.get_action_size(agent_id)
    	  out_offset = [0.0000000000,0.0000000000,0.0000000000,-0.200000000,0.0000000000,0.0000000000,0.0000000000,
    	    -0.200000000,0.0000000000,0.0000000000,	0.00000000,	-0.2000000,	1.57000000,	0.00000000,	0.00000000,
    	    0.00000000,	-0.2000000,	0.00000000,	0.00000000,	0.00000000,	-0.2000000,	-1.5700000,	0.00000000,	0.00000000,
    	    0.00000000,	-0.2000000,	1.57000000,	0.00000000,	0.00000000,	0.00000000,	-0.2000000,	0.00000000,	0.00000000,
    	    0.00000000,	-0.2000000,	-1.5700000]
    	  #see cCtCtrlUtil::BuildOffsetScalePDPrismatic and
    	  #see cCtCtrlUtil::BuildOffsetScalePDSpherical
    	  return np.array(out_offset)
    	  
    def build_action_scale(self, agent_id):
    	  out_scale = [1] * self.get_action_size(agent_id)
    	  #see cCtCtrlUtil::BuildOffsetScalePDPrismatic and
    	  #see cCtCtrlUtil::BuildOffsetScalePDSpherical
    	  out_scale=[ 0.20833333333333,1.00000000000000,1.00000000000000,1.00000000000000,0.25000000000000,
    	    1.00000000000000,1.00000000000000,1.00000000000000,0.12077294685990,1.00000000000000,
    	    1.000000000000,	1.000000000000,	0.159235668789,	0.159235668789,	1.000000000000,
    	    1.000000000000,	1.000000000000,	0.079617834394,	1.000000000000,	1.000000000000,
    	    1.000000000000,	0.159235668789,	0.120772946859,	1.000000000000,	1.000000000000,
    	    1.000000000000,	0.159235668789,	0.159235668789,	1.000000000000,	1.000000000000,
    	    1.000000000000,	0.107758620689,	1.000000000000,	1.000000000000,	1.000000000000,
    	    0.159235668789]
    	  return np.array(out_scale)
    
    def build_action_bound_min(self, agent_id):
    	  #see cCtCtrlUtil::BuildBoundsPDSpherical
    	  out_scale = [-1] * self.get_action_size(agent_id)
    	  out_scale = [-4.79999999999,-1.00000000000,-1.00000000000,-1.00000000000,-4.00000000000,
    	    -1.00000000000,-1.00000000000,-1.00000000000,-7.77999999999,-1.00000000000,	-1.000000000,
	    	  -1.000000000,	-7.850000000,	-6.280000000,	-1.000000000,	-1.000000000,	-1.000000000,
	    	  -12.56000000,	-1.000000000,	-1.000000000,	-1.000000000,	-4.710000000,
	    	  -7.779999999,	-1.000000000,	-1.000000000,	-1.000000000,	-7.850000000,
	    	  -6.280000000,	-1.000000000,	-1.000000000,	-1.000000000,	-8.460000000,
	    	  -1.000000000,	-1.000000000,	-1.000000000,	-4.710000000]

    	  return out_scale
    
    def build_action_bound_max(self, agent_id):
    	  out_scale = [1] * self.get_action_size(agent_id)
    	  out_scale=[
    	    4.799999999,1.000000000,1.000000000,1.000000000,4.000000000,1.000000000,
    	    1.000000000,1.000000000,8.779999999,1.000000000,	1.0000000,	1.0000000,
    	  	4.7100000,	6.2800000,	1.0000000,	1.0000000,	1.0000000,
	    	  12.560000,	1.0000000,	1.0000000,	1.0000000,	7.8500000,
	    	  8.7799999,	1.0000000,	1.0000000,	1.0000000,	4.7100000,
	    	  6.2800000,	1.0000000,	1.0000000,	1.0000000,	10.100000,
	    	  1.0000000,	1.0000000,	1.0000000,	7.8500000]
    	  return out_scale
    	  
    def set_mode(self, mode):
    	  self._mode = mode
    	  
    def need_new_action(self, agent_id):
        return True
        
    def record_state(self, agent_id):
        state = self._humanoid.getState()
        state[1]=state[1]+0.008
        #print("record_state=",state)
        return np.array(state)
        
        
    def record_goal(self, agent_id):
        return np.array([])
        
    def calc_reward(self, agent_id):
        kinPose = self._humanoid.computePose(self._humanoid._frameFraction)
        reward = self._humanoid.getReward(kinPose)
        return reward
        
    def set_action(self, agent_id, action):
        self.desiredPose = self._humanoid.convertActionToPose(action)
        #print("set_action: desiredPose=", self.desiredPose)
        
    def log_val(self, agent_id, val):
        pass
        
    def update(self, timeStep):
        for i in range(10):
          self.t += timeStep
          self._humanoid.setSimTime(self.t)
          
          if self.desiredPose:
            kinPose = self._humanoid.computePose(self._humanoid._frameFraction)
            self._humanoid.initializePose(self._humanoid._poseInterpolator, self._humanoid._kin_model, initBase=False)
            pos,orn=self._pybullet_client.getBasePositionAndOrientation(self._humanoid._sim_model)
            self._pybullet_client.resetBasePositionAndOrientation(self._humanoid._kin_model, [pos[0],pos[1]+2,pos[2]],orn)
            #print("desiredPositions=",self.desiredPose)
            taus = self._humanoid.computePDForces(self.desiredPose)
            self._humanoid.applyPDForces(taus)
            self._pybullet_client.stepSimulation()
            

    def set_sample_count(self, count):
        return
        
    def check_terminate(self, agent_id):
      return Env.Terminate(self.is_episode_end())
      
    def is_episode_end(self):
      isEnded = self._humanoid.terminates()
      #also check maximum time, 20 seconds (todo get from file)
      #print("self.t=",self.t)
      if (self.t>3):
        isEnded = True
      return isEnded
      
    def check_valid_episode(self):
      #could check if limbs exceed velocity threshold
      return true
