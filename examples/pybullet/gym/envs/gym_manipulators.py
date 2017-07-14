from pybulletgym.envs.scene_abstract import SingleRobotEmptyScene
from gym_mujoco_xml_env import PybulletMujocoXmlEnv
import gym, gym.spaces, gym.utils, gym.utils.seeding
import numpy as np
import os, sys

class PybulletReacher(PybulletMujocoXmlEnv):
	def __init__(self):
		PybulletMujocoXmlEnv.__init__(self, 'reacher.xml', 'body0', action_dim=2, obs_dim=9)

	def create_single_player_scene(self):
		return SingleRobotEmptyScene(gravity=0.0, timestep=0.0165, frame_skip=1)

	TARG_LIMIT = 0.27
	def robot_specific_reset(self):
		self.jdict["target_x"].reset_current_position(self.np_random.uniform( low=-self.TARG_LIMIT, high=self.TARG_LIMIT ), 0)
		self.jdict["target_y"].reset_current_position(self.np_random.uniform( low=-self.TARG_LIMIT, high=self.TARG_LIMIT ), 0)
		self.fingertip = self.parts["fingertip"]
		self.target	= self.parts["target"]
		self.central_joint = self.jdict["joint0"]
		self.elbow_joint   = self.jdict["joint1"]
		self.central_joint.reset_current_position(self.np_random.uniform( low=-3.14, high=3.14 ), 0)
		self.elbow_joint.reset_current_position(self.np_random.uniform( low=-3.14, high=3.14 ), 0)

	def apply_action(self, a):
		assert( np.isfinite(a).all() )
		self.central_joint.set_motor_torque( 0.05*float(np.clip(a[0], -1, +1)) )
		self.elbow_joint.set_motor_torque( 0.05*float(np.clip(a[1], -1, +1)) )

	def calc_state(self):
		theta,	 self.theta_dot = self.central_joint.current_relative_position()
		self.gamma, self.gamma_dot = self.elbow_joint.current_relative_position()
		target_x, _ = self.jdict["target_x"].current_position()
		target_y, _ = self.jdict["target_y"].current_position()
		self.to_target_vec = np.array(self.fingertip.pose().xyz()) - np.array(self.target.pose().xyz())
		return np.array([
			target_x,
			target_y,
			self.to_target_vec[0],
			self.to_target_vec[1],
			np.cos(theta),
			np.sin(theta),
			self.theta_dot,
			self.gamma,
			self.gamma_dot,
			])

	def calc_potential(self):
		return -100 * np.linalg.norm(self.to_target_vec)

	def _step(self, a):
		assert(not self.scene.multiplayer)
		self.apply_action(a)
		self.scene.global_step()

		state = self.calc_state()  # sets self.to_target_vec

		potential_old = self.potential
		self.potential = self.calc_potential()

		electricity_cost = (
			-0.10*(np.abs(a[0]*self.theta_dot) + np.abs(a[1]*self.gamma_dot))  # work torque*angular_velocity
			-0.01*(np.abs(a[0]) + np.abs(a[1]))								# stall torque require some energy
			)
		stuck_joint_cost = -0.1 if np.abs(np.abs(self.gamma)-1) < 0.01 else 0.0
		self.rewards = [float(self.potential - potential_old), float(electricity_cost), float(stuck_joint_cost)]
		self.HUD(state, a, False)
		return state, sum(self.rewards), False, {}

	def camera_adjust(self):
		x, y, z = self.fingertip.pose().xyz()
		x *= 0.5
		y *= 0.5
		self.camera.move_and_look_at(0.3, 0.3, 0.3, x, y, z)


class PybulletPusher(PybulletMujocoXmlEnv):
	def __init__(self):
		PybulletMujocoXmlEnv.__init__(self, 'pusher.xml', 'body0', action_dim=7, obs_dim=5)

	def create_single_player_scene(self):
		return SingleRobotEmptyScene(gravity=9.81, timestep=0.0020, frame_skip=5)

	def robot_specific_reset(self):
		self.fingertip = self.parts["fingertip"]
#		qpos = self.init_qpos

		self.goal_pos = np.asarray([0, 0])
		while True:
			self.cylinder_pos = np.concatenate([
					self.np_random.uniform(low=-0.3, high=0, size=1),
					self.np_random.uniform(low=-0.2, high=0.2, size=1)])
			if np.linalg.norm(self.cylinder_pos - self.goal_pos) > 0.17:
				break

# This is probably position setting
#		qpos[-4:-2] = self.cylinder_pos
#		qpos[-2:] = self.goal_pos
#		qvel = self.init_qvel + self.np_random.uniform(low=-0.005,
#				high=0.005, size=self.model.nv)
#		qvel[-4:] = 0
#		self.set_state(qpos, qvel)

	def apply_action(self, a):
		assert( np.isfinite(a).all() )

	def calc_state(self):
		return np.concatenate([
			np.array([j.current_position() for j in self.ordered_joints]).flatten(), # position
			np.array([j.current_relative_position() for j in self.ordered_joints]).flatten(), # speed
			self.parts["fingertip"].pose().xyz(),
			self.parts["object"].pose().xyz(),
			self.parts["goal"].pose().xyz(),
		])

	def _step(self, a):

		self.apply_action(a)
		self.scene.global_step()

		state = self.calc_state()

		reward_near_vec = self.parts["object"].pose().xyz() - self.parts["fingertip"].pose().xyz()
		reward_dist_vec = self.parts["object"].pose().xyz() - self.parts["goal"].pose().xyz()

		reward_near = - np.linalg.norm(reward_near_vec)
		reward_dist = - np.linalg.norm(reward_dist_vec)
		reward_ctrl = - np.square(a).sum()
		reward = reward_dist + 0.1 * reward_ctrl + 0.5 * reward_near

		done = False
		return state, reward, done, dict(reward_dist=reward_dist, reward_ctrl=reward_ctrl)

	def camera_adjust(self):
		x, y, z = self.fingertip.pose().xyz()
		x *= 0.5
		y *= 0.5
		self.camera.move_and_look_at(0.3, 0.3, 0.3, x, y, z)


class PybulletStriker(PybulletMujocoXmlEnv):
	def __init__(self):
		PybulletMujocoXmlEnv.__init__(self, 'striker.xml', 'body0', action_dim=7, obs_dim=5)
		self._striked = False
		self._min_strike_dist = np.inf
		self.strike_threshold = 0.1

	def create_single_player_scene(self):
		return SingleRobotEmptyScene(gravity=9.81, timestep=0.0020, frame_skip=5)

	def robot_specific_reset(self):
		self.fingertip = self.parts["fingertip"]
		self._min_strike_dist = np.inf
		self._striked = False
		self._strike_pos = None
		
		# reset position of manipulator
		for j in self.ordered_joints:
			j.reset_current_position(self.np_random.uniform( low=-0.1, high=0.1 ))
			
		# reset speed of manipulator
		
		# reset ball position

#		qpos = self.init_qpos

		self.ball = np.array([0.5, -0.175])

		while True:
			self.goal = np.concatenate([
					self.np_random.uniform(low=0.15, high=0.7, size=1),
					self.np_random.uniform(low=0.1, high=1.0, size=1)])
			if np.linalg.norm(self.ball - self.goal) > 0.17:
				break
# This is probably position setting
#		qpos[-9:-7] = [self.ball[1], self.ball[0]]
#		qpos[-7:-5] = self.goal
#		diff = self.ball - self.goal
#		angle = -np.arctan(diff[0] / (diff[1] + 1e-8))
#		qpos[-1] = angle / 3.14
#		qvel = self.init_qvel + self.np_random.uniform(low=-.1, high=.1,
#				size=self.model.nv)
#		qvel[7:] = 0
#		self.set_state(qpos, qvel)

	def apply_action(self, a):
		assert( np.isfinite(a).all() )

	def calc_state(self):
		return np.concatenate([
			np.array([j.current_position() for j in self.ordered_joints]).flatten(), # position
			np.array([j.current_relative_position() for j in self.ordered_joints]).flatten(), # speed
			self.parts["fingertip"].pose().xyz(),
			self.parts["object"].pose().xyz(),
			self.parts["goal"].pose().xyz(),
		])

	def _step(self, a):
		self.apply_action(a)
		self.scene.global_step()
		state = self.calc_state()

		dist_object_finger = self.parts["object"].pose().xyz() - self.parts["fingertip"].pose().xyz()
		reward_dist_vec = self.parts["object"].pose().xyz() - self.parts["goal"].pose().xyz()

		self._min_strike_dist = min(self._min_strike_dist, np.linalg.norm(reward_dist_vec))

		if np.linalg.norm(dist_object_finger) < self.strike_threshold:
			self._striked = True
			self._strike_pos = self.parts["fingertip"].pose().xyz()

		if self._striked:
			reward_near_vec = self.parts["object"].pose().xyz() - self._strike_pos
		else:
			reward_near_vec = self.parts["object"].pose().xyz() - self.parts["fingertip"].pose().xyz()

		reward_near = - np.linalg.norm(reward_near_vec)

		reward_dist = - np.linalg.norm(self._min_strike_dist)
		reward_ctrl = - np.square(a).sum()
		reward = 3 * reward_dist + 0.1 * reward_ctrl + 0.5 * reward_near

		done = False
		return state, reward, done, dict(reward_dist=reward_dist, reward_ctrl=reward_ctrl)

	def camera_adjust(self):
		x, y, z = self.fingertip.pose().xyz()
		x *= 0.5
		y *= 0.5
		self.camera.move_and_look_at(0.3, 0.3, 0.3, x, y, z)


class PybulletThrower(PybulletMujocoXmlEnv):
	def __init__(self):
		PybulletMujocoXmlEnv.__init__(self, 'thrower.xml', 'body0', action_dim=7, obs_dim=5)
		self._ball_hit_ground = False
		self._ball_hit_location = None

	def create_single_player_scene(self):
		return SingleRobotEmptyScene(gravity=0.0, timestep=0.0020, frame_skip=5)

	def robot_specific_reset(self):
		self.fingertip = self.parts["fingertip"]
		self._ball_hit_ground = False
		self._ball_hit_location = None
		
		# reset position of manipulator
		for j in self.ordered_joints:
			j.reset_current_position(self.np_random.uniform( low=-0.1, high=0.1 ))
			
		# reset speed of manipulator
# This is probably position setting
#		qpos = self.init_qpos
#		self.goal = np.array([self.np_random.uniform(low=-0.3, high=0.3),
#							  self.np_random.uniform(low=-0.3, high=0.3)])
# 
#		qpos[-9:-7] = self.goal
#		qvel = self.init_qvel + self.np_random.uniform(low=-0.005,
#				high=0.005, size=self.model.nv)
#		qvel[7:] = 0
#		self.set_state(qpos, qvel)

	def apply_action(self, a):
		assert( np.isfinite(a).all() )

	def calc_state(self):
		return np.concatenate([
			np.array([j.current_position() for j in self.ordered_joints]).flatten(), # position
			np.array([j.current_relative_position() for j in self.ordered_joints]).flatten(), # speed
			self.parts["fingertip"].pose().xyz(),
			self.parts["ball"].pose().xyz(),
			self.parts["goal"].pose().xyz(),
		])

	def _step(self, a):
		self.apply_action(a)
		self.scene.global_step()
		state = self.calc_state()

		ball_xy = self.parts["ball"].pose().xyz()[:2]
		goal_xy = self.parts["goal"].pose().xyz()[:2]

		if not self._ball_hit_ground and self.parts["ball"].pose().xyz()[2] < -0.25:
			self._ball_hit_ground = True
			self._ball_hit_location = self.parts["ball"].pose().xyz()

		if self._ball_hit_ground:
			ball_hit_xy = self._ball_hit_location[:2]
			reward_dist = -np.linalg.norm(ball_hit_xy - goal_xy)
		else:
			reward_dist = -np.linalg.norm(ball_xy - goal_xy)
		reward_ctrl = - np.square(a).sum()

		reward = reward_dist + 0.002 * reward_ctrl

		done = False
		return state, reward, done, dict(reward_dist=reward_dist, reward_ctrl=reward_ctrl)

	def camera_adjust(self):
		x, y, z = self.fingertip.pose().xyz()
		x *= 0.5
		y *= 0.5
		self.camera.move_and_look_at(0.3, 0.3, 0.3, x, y, z)

