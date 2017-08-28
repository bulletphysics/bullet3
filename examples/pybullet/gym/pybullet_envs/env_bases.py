import gym, gym.spaces, gym.utils, gym.utils.seeding
import numpy as np
import pybullet as p


class MJCFBaseBulletEnv(gym.Env):
	"""
	Base class for Bullet physics simulation loading MJCF (MuJoCo .xml) environments in a Scene.
	These environments create single-player scenes and behave like normal Gym environments, if
	you don't use multiplayer.
	"""

	metadata = {
		'render.modes': ['human', 'rgb_array'],
		'video.frames_per_second': 60
		}

	def __init__(self, robot, render=False):
		self.scene = None
		self.physicsClientId=-1
		self.camera = Camera()
		self.isRender = render
		self.robot = robot

		self._seed()

		self.action_space = robot.action_space
		self.observation_space = robot.observation_space

	def _seed(self, seed=None):
		self.np_random, seed = gym.utils.seeding.np_random(seed)
		self.robot.np_random = self.np_random # use the same np_randomizer for robot as for env
		return [seed]

	def _reset(self):
		print("self.isRender=")
		print(self.isRender)
		if (self.physicsClientId<0):
			if (self.isRender):
				self.physicsClientId = p.connect(p.GUI)
			else:
				self.physicsClientId = p.connect(p.DIRECT)
		p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
  
		if self.scene is None:
			self.scene = self.create_single_player_scene()
		if not self.scene.multiplayer:
			self.scene.episode_restart()

		self.robot.scene = self.scene

		self.frame = 0
		self.done = 0
		self.reward = 0
		dump = 0
		s = self.robot.reset()
		self.potential = self.robot.calc_potential()
		return s

	def _render(self, mode, close):
		if (mode=="human"):
			self.isRender = True

	def _close(self):
		if (self.physicsClientId>=0):
			p.disconnect(self.physicsClientId)
			self.physicsClientId = -1

	def HUD(self, state, a, done):
		pass

class Camera:
	def __init__(self):
		pass

	def move_and_look_at(self,i,j,k,x,y,z):
		lookat = [x,y,z]
		distance = 10
		yaw = 10
		p.resetDebugVisualizerCamera(distance, yaw, -20, lookat)
