from .scene_abstract import SingleRobotEmptyScene
from .env_bases import MJCFBaseBulletEnv
from robot_pendula import InvertedPendulum, InvertedPendulumSwingup, InvertedDoublePendulum
import gym, gym.spaces, gym.utils, gym.utils.seeding
import numpy as np
import pybullet 
import os, sys

class InvertedPendulumBulletEnv(MJCFBaseBulletEnv):
	def __init__(self):
		self.robot = InvertedPendulum()
		MJCFBaseBulletEnv.__init__(self, self.robot)
		self.stateId=-1

	def create_single_player_scene(self, bullet_client):
		return SingleRobotEmptyScene(bullet_client, gravity=9.8, timestep=0.0165, frame_skip=1)

	def _reset(self):
		if (self.stateId>=0):
			#print("InvertedPendulumBulletEnv reset p.restoreState(",self.stateId,")")
			self._p.restoreState(self.stateId)
		r = MJCFBaseBulletEnv._reset(self)
		if (self.stateId<0):
			self.stateId = self._p.saveState()
			#print("InvertedPendulumBulletEnv reset self.stateId=",self.stateId)
		return r
	
	def _step(self, a):
		self.robot.apply_action(a)
		self.scene.global_step()
		state = self.robot.calc_state()  # sets self.pos_x self.pos_y
		vel_penalty = 0
		if self.robot.swingup:
			reward = np.cos(self.robot.theta)
			done = False
		else:
			reward = 1.0
			done = np.abs(self.robot.theta) > .2
		self.rewards = [float(reward)]
		self.HUD(state, a, done)
		return state, sum(self.rewards), done, {}

	def camera_adjust(self):
		self.camera.move_and_look_at(0,1.2,1.0, 0,0,0.5)

class InvertedPendulumSwingupBulletEnv(InvertedPendulumBulletEnv):
	def __init__(self):
		self.robot = InvertedPendulumSwingup()
		MJCFBaseBulletEnv.__init__(self, self.robot)
		self.stateId=-1

class InvertedDoublePendulumBulletEnv(MJCFBaseBulletEnv):
	def __init__(self):
		self.robot = InvertedDoublePendulum()
		MJCFBaseBulletEnv.__init__(self, self.robot)
		self.stateId = -1
	def create_single_player_scene(self, bullet_client):
		return SingleRobotEmptyScene(bullet_client, gravity=9.8, timestep=0.0165, frame_skip=1)

	def _reset(self):
		if (self.stateId>=0):
			self._p.restoreState(self.stateId)
		r = MJCFBaseBulletEnv._reset(self)
		if (self.stateId<0):
			self.stateId = self._p.saveState()
		return r
	
	def _step(self, a):
		self.robot.apply_action(a)
		self.scene.global_step()
		state = self.robot.calc_state()  # sets self.pos_x self.pos_y
		# upright position: 0.6 (one pole) + 0.6 (second pole) * 0.5 (middle of second pole) = 0.9
		# using <site> tag in original xml, upright position is 0.6 + 0.6 = 1.2, difference +0.3
		dist_penalty = 0.01 * self.robot.pos_x ** 2 + (self.robot.pos_y + 0.3 - 2) ** 2
		# v1, v2 = self.model.data.qvel[1:3]   TODO when this fixed https://github.com/bulletphysics/bullet3/issues/1040
		#vel_penalty = 1e-3 * v1**2 + 5e-3 * v2**2
		vel_penalty = 0
		alive_bonus = 10
		done = self.robot.pos_y + 0.3 <= 1
		self.rewards = [float(alive_bonus), float(-dist_penalty), float(-vel_penalty)]
		self.HUD(state, a, done)
		return state, sum(self.rewards), done, {}

	def camera_adjust(self):
		self.camera.move_and_look_at(0,1.2,1.2, 0,0,0.5)
