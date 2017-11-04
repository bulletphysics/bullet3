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
		self._cam_dist = 3
		self._cam_yaw = 0
		self._cam_pitch = -30
		self._render_width =320
		self._render_height = 240

		self.action_space = robot.action_space
		self.observation_space = robot.observation_space
	def configure(self, args):
		self.robot.args = args
	def _seed(self, seed=None):
		self.np_random, seed = gym.utils.seeding.np_random(seed)
		self.robot.np_random = self.np_random # use the same np_randomizer for robot as for env
		return [seed]

	def _reset(self):
		if (self.physicsClientId<0):
			self.physicsClientId = p.connect(p.SHARED_MEMORY)
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
		if mode != "rgb_array":
			return np.array([])
		
		base_pos=[0,0,0]
		if (hasattr(self,'robot')):
			if (hasattr(self.robot,'body_xyz')):
				base_pos = self.robot.body_xyz
		
		view_matrix = p.computeViewMatrixFromYawPitchRoll(
			cameraTargetPosition=base_pos,
			distance=self._cam_dist,
			yaw=self._cam_yaw,
			pitch=self._cam_pitch,
			roll=0,
			upAxisIndex=2)
		proj_matrix = p.computeProjectionMatrixFOV(
			fov=60, aspect=float(self._render_width)/self._render_height,
			nearVal=0.1, farVal=100.0)
		(_, _, px, _, _) = p.getCameraImage(
		width=self._render_width, height=self._render_height, viewMatrix=view_matrix,
			projectionMatrix=proj_matrix,
			renderer=p.ER_BULLET_HARDWARE_OPENGL
			)
		rgb_array = np.array(px)
		rgb_array = rgb_array[:, :, :3]
		return rgb_array

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
