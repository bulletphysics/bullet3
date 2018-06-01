from robot_bases import XmlBasedRobot, MJCFBasedRobot, URDFBasedRobot
import numpy as np
import pybullet 
import os
import pybullet_data
from robot_bases import BodyPart

class WalkerBase(MJCFBasedRobot):
	def __init__(self,  fn, robot_name, action_dim, obs_dim, power):
		MJCFBasedRobot.__init__(self, fn, robot_name, action_dim, obs_dim)
		self.power = power
		self.camera_x = 0
		self.start_pos_x, self.start_pos_y, self.start_pos_z = 0, 0, 0
		self.walk_target_x = 1e3  # kilometer away
		self.walk_target_y = 0
		self.body_xyz=[0,0,0]

	def robot_specific_reset(self, bullet_client):
		self._p = bullet_client
		for j in self.ordered_joints:
			j.reset_current_position(self.np_random.uniform(low=-0.1, high=0.1), 0)

		self.feet = [self.parts[f] for f in self.foot_list]
		self.feet_contact = np.array([0.0 for f in self.foot_list], dtype=np.float32)
		self.scene.actor_introduce(self)
		self.initial_z = None

	def apply_action(self, a):
		assert (np.isfinite(a).all())
		for n, j in enumerate(self.ordered_joints):
			j.set_motor_torque(self.power * j.power_coef * float(np.clip(a[n], -1, +1)))

	def calc_state(self):
		j = np.array([j.current_relative_position() for j in self.ordered_joints], dtype=np.float32).flatten()
		# even elements [0::2] position, scaled to -1..+1 between limits
		# odd elements  [1::2] angular speed, scaled to show -1..+1
		self.joint_speeds = j[1::2]
		self.joints_at_limit = np.count_nonzero(np.abs(j[0::2]) > 0.99)

		body_pose = self.robot_body.pose()
		parts_xyz = np.array([p.pose().xyz() for p in self.parts.values()]).flatten()
		self.body_xyz = (
		parts_xyz[0::3].mean(), parts_xyz[1::3].mean(), body_pose.xyz()[2])  # torso z is more informative than mean z
		self.body_rpy = body_pose.rpy()
		z = self.body_xyz[2]
		if self.initial_z == None:
			self.initial_z = z
		r, p, yaw = self.body_rpy
		self.walk_target_theta = np.arctan2(self.walk_target_y - self.body_xyz[1],
											self.walk_target_x - self.body_xyz[0])
		self.walk_target_dist = np.linalg.norm(
			[self.walk_target_y - self.body_xyz[1], self.walk_target_x - self.body_xyz[0]])
		angle_to_target = self.walk_target_theta - yaw

		rot_speed = np.array(
			[[np.cos(-yaw), -np.sin(-yaw), 0],
			 [np.sin(-yaw), np.cos(-yaw), 0],
			 [		0,			 0, 1]]
		)
		vx, vy, vz = np.dot(rot_speed, self.robot_body.speed())  # rotate speed back to body point of view

		more = np.array([ z-self.initial_z,
			np.sin(angle_to_target), np.cos(angle_to_target),
			0.3* vx , 0.3* vy , 0.3* vz ,  # 0.3 is just scaling typical speed into -1..+1, no physical sense here
			r, p], dtype=np.float32)
		return np.clip( np.concatenate([more] + [j] + [self.feet_contact]), -5, +5)

	def calc_potential(self):
		# progress in potential field is speed*dt, typical speed is about 2-3 meter per second, this potential will change 2-3 per frame (not per second),
		# all rewards have rew/frame units and close to 1.0
		debugmode=0
		if (debugmode):
			print("calc_potential: self.walk_target_dist")
			print(self.walk_target_dist)
			print("self.scene.dt")
			print(self.scene.dt)
			print("self.scene.frame_skip")
			print(self.scene.frame_skip)
			print("self.scene.timestep")
			print(self.scene.timestep)
		return - self.walk_target_dist / self.scene.dt


class Hopper(WalkerBase):
	foot_list = ["foot"]

	def __init__(self):
		WalkerBase.__init__(self, "hopper.xml", "torso", action_dim=3, obs_dim=15, power=0.75)

	def alive_bonus(self, z, pitch):
		return +1 if z > 0.8 and abs(pitch) < 1.0 else -1


class Walker2D(WalkerBase):
	foot_list = ["foot", "foot_left"]

	def __init__(self):
		WalkerBase.__init__(self,  "walker2d.xml", "torso", action_dim=6, obs_dim=22, power=0.40)

	def alive_bonus(self, z, pitch):
		return +1 if z > 0.8 and abs(pitch) < 1.0 else -1

	def robot_specific_reset(self, bullet_client):
		WalkerBase.robot_specific_reset(self, bullet_client)
		for n in ["foot_joint", "foot_left_joint"]:
			self.jdict[n].power_coef = 30.0


class HalfCheetah(WalkerBase):
	foot_list = ["ffoot", "fshin", "fthigh",  "bfoot", "bshin", "bthigh"]  # track these contacts with ground

	def __init__(self):
		WalkerBase.__init__(self, "half_cheetah.xml", "torso", action_dim=6, obs_dim=26, power=0.90)

	def alive_bonus(self, z, pitch):
		# Use contact other than feet to terminate episode: due to a lot of strange walks using knees
		return +1 if np.abs(pitch) < 1.0 and not self.feet_contact[1] and not self.feet_contact[2] and not self.feet_contact[4] and not self.feet_contact[5] else -1

	def robot_specific_reset(self, bullet_client):
		WalkerBase.robot_specific_reset(self, bullet_client)
		self.jdict["bthigh"].power_coef = 120.0
		self.jdict["bshin"].power_coef  = 90.0
		self.jdict["bfoot"].power_coef  = 60.0
		self.jdict["fthigh"].power_coef = 140.0
		self.jdict["fshin"].power_coef  = 60.0
		self.jdict["ffoot"].power_coef  = 30.0


class Ant(WalkerBase):
	foot_list = ['front_left_foot', 'front_right_foot', 'left_back_foot', 'right_back_foot']

	def __init__(self):
		WalkerBase.__init__(self, "ant.xml", "torso", action_dim=8, obs_dim=28, power=2.5)

	def alive_bonus(self, z, pitch):
		return +1 if z > 0.26 else -1  # 0.25 is central sphere rad, die if it scrapes the ground


class Humanoid(WalkerBase):
	self_collision = True
	foot_list = ["right_foot", "left_foot"]  # "left_hand", "right_hand"

	def __init__(self):
		WalkerBase.__init__(self,  'humanoid_symmetric.xml', 'torso', action_dim=17, obs_dim=44, power=0.41)
		# 17 joints, 4 of them important for walking (hip, knee), others may as well be turned off, 17/4 = 4.25

	def robot_specific_reset(self, bullet_client):
		WalkerBase.robot_specific_reset(self, bullet_client)
		self.motor_names  = ["abdomen_z", "abdomen_y", "abdomen_x"]
		self.motor_power  = [100, 100, 100]
		self.motor_names += ["right_hip_x", "right_hip_z", "right_hip_y", "right_knee"]
		self.motor_power += [100, 100, 300, 200]
		self.motor_names += ["left_hip_x", "left_hip_z", "left_hip_y", "left_knee"]
		self.motor_power += [100, 100, 300, 200]
		self.motor_names += ["right_shoulder1", "right_shoulder2", "right_elbow"]
		self.motor_power += [75, 75, 75]
		self.motor_names += ["left_shoulder1", "left_shoulder2", "left_elbow"]
		self.motor_power += [75, 75, 75]
		self.motors = [self.jdict[n] for n in self.motor_names]
		if self.random_yaw:
			position = [0,0,0]
			orientation = [0,0,0]
			yaw = self.np_random.uniform(low=-3.14, high=3.14)
			if self.random_lean and self.np_random.randint(2)==0:
				cpose.set_xyz(0, 0, 1.4)
				if self.np_random.randint(2)==0:
					pitch = np.pi/2
					position = [0, 0, 0.45]
				else:
					pitch = np.pi*3/2
					position = [0, 0, 0.25]
				roll = 0
				orientation = [roll, pitch, yaw]
			else:
				position = [0, 0, 1.4]
				orientation = [0, 0, yaw]  # just face random direction, but stay straight otherwise
			self.robot_body.reset_position(position)
			self.robot_body.reset_orientation(orientation)
		self.initial_z = 0.8

	random_yaw = False
	random_lean = False

	def apply_action(self, a):
		assert( np.isfinite(a).all() )
		force_gain = 1
		for i, m, power in zip(range(17), self.motors, self.motor_power):
			m.set_motor_torque(float(force_gain * power * self.power * np.clip(a[i], -1, +1)))

	def alive_bonus(self, z, pitch):
		return +2 if z > 0.78 else -1   # 2 here because 17 joints produce a lot of electricity cost just from policy noise, living must be better than dying




def get_cube(_p, x, y, z):	
	body = _p.loadURDF(os.path.join(pybullet_data.getDataPath(),"cube_small.urdf"), [x, y, z])
	_p.changeDynamics(body,-1, mass=1.2)#match Roboschool
	part_name, _ = _p.getBodyInfo(body)
	part_name = part_name.decode("utf8")
	bodies = [body]
	return BodyPart(_p, part_name, bodies, 0, -1)


def get_sphere(_p, x, y, z):
	body = _p.loadURDF(os.path.join(pybullet_data.getDataPath(),"sphere2red_nocol.urdf"), [x, y, z])
	part_name, _ = _p.getBodyInfo(body)
	part_name = part_name.decode("utf8")
	bodies = [body]
	return BodyPart(_p, part_name, bodies, 0, -1)

class HumanoidFlagrun(Humanoid):
	def __init__(self):
		Humanoid.__init__(self)
		self.flag = None
		
	def robot_specific_reset(self, bullet_client):
		Humanoid.robot_specific_reset(self, bullet_client)
		self.flag_reposition()

	def flag_reposition(self):
		self.walk_target_x = self.np_random.uniform(low=-self.scene.stadium_halflen,   high=+self.scene.stadium_halflen)
		self.walk_target_y = self.np_random.uniform(low=-self.scene.stadium_halfwidth, high=+self.scene.stadium_halfwidth)
		more_compact = 0.5  # set to 1.0 whole football field
		self.walk_target_x *= more_compact
		self.walk_target_y *= more_compact
		
		if (self.flag):
			#for b in self.flag.bodies:
			#	print("remove body uid",b)
			#	p.removeBody(b)
			self._p.resetBasePositionAndOrientation(self.flag.bodies[0],[self.walk_target_x, self.walk_target_y, 0.7],[0,0,0,1])
		else:
			self.flag = get_sphere(self._p, self.walk_target_x, self.walk_target_y, 0.7)
		self.flag_timeout = 600/self.scene.frame_skip #match Roboschool 

	def calc_state(self):
		self.flag_timeout -= 1
		state = Humanoid.calc_state(self)
		if self.walk_target_dist < 1 or self.flag_timeout <= 0:
			self.flag_reposition()
			state = Humanoid.calc_state(self)  # caclulate state again, against new flag pos
			self.potential = self.calc_potential()	   # avoid reward jump
		return state


class HumanoidFlagrunHarder(HumanoidFlagrun):
	def __init__(self):
		HumanoidFlagrun.__init__(self)
		self.flag = None
		self.aggressive_cube = None
		self.frame = 0

	def robot_specific_reset(self, bullet_client):

		HumanoidFlagrun.robot_specific_reset(self, bullet_client)
		
		self.frame = 0
		if (self.aggressive_cube):
			self._p.resetBasePositionAndOrientation(self.aggressive_cube.bodies[0],[-1.5,0,0.05],[0,0,0,1])
		else:
			self.aggressive_cube = get_cube(self._p, -1.5,0,0.05)
		self.on_ground_frame_counter = 0
		self.crawl_start_potential = None
		self.crawl_ignored_potential = 0.0
		self.initial_z = 0.8

	def alive_bonus(self, z, pitch):
		if self.frame%30==0 and self.frame>100 and self.on_ground_frame_counter==0:
			target_xyz  = np.array(self.body_xyz)
			robot_speed = np.array(self.robot_body.speed())
			angle = self.np_random.uniform(low=-3.14, high=3.14)
			from_dist   = 4.0
			attack_speed   = self.np_random.uniform(low=20.0, high=30.0)  # speed 20..30 (* mass in cube.urdf = impulse)
			time_to_travel = from_dist / attack_speed
			target_xyz += robot_speed*time_to_travel  # predict future position at the moment the cube hits the robot
			position = [target_xyz[0] + from_dist*np.cos(angle),
				target_xyz[1] + from_dist*np.sin(angle),
				target_xyz[2] + 1.0]
			attack_speed_vector = target_xyz - np.array(position)
			attack_speed_vector *= attack_speed / np.linalg.norm(attack_speed_vector)
			attack_speed_vector += self.np_random.uniform(low=-1.0, high=+1.0, size=(3,))
			self.aggressive_cube.reset_position(position)
			self.aggressive_cube.reset_velocity(linearVelocity=attack_speed_vector)
		if z < 0.8:
			self.on_ground_frame_counter += 1
		elif self.on_ground_frame_counter > 0:
			self.on_ground_frame_counter -= 1
		# End episode if the robot can't get up in 170 frames, to save computation and decorrelate observations.
		self.frame += 1
		return self.potential_leak() if self.on_ground_frame_counter<170 else -1

	def potential_leak(self):
		z = self.body_xyz[2]		  # 0.00 .. 0.8 .. 1.05 normal walk, 1.2 when jumping
		z = np.clip(z, 0, 0.8)
		return z/0.8 + 1.0			# 1.00 .. 2.0

	def calc_potential(self):
		# We see alive bonus here as a leak from potential field. Value V(s) of a given state equals
		# potential, if it is topped up with gamma*potential every frame. Gamma is assumed 0.99.
		#
		# 2.0 alive bonus if z>0.8, potential is 200, leak gamma=0.99, (1-0.99)*200==2.0
		# 1.0 alive bonus on the ground z==0, potential is 100, leak (1-0.99)*100==1.0
		#
		# Why robot whould stand up: to receive 100 points in potential field difference.
		flag_running_progress = Humanoid.calc_potential(self)

		# This disables crawl.
		if self.body_xyz[2] < 0.8:
			if self.crawl_start_potential is None:
				self.crawl_start_potential = flag_running_progress - self.crawl_ignored_potential
				#print("CRAWL START %+0.1f %+0.1f" % (self.crawl_start_potential, flag_running_progress))
			self.crawl_ignored_potential = flag_running_progress - self.crawl_start_potential
			flag_running_progress  = self.crawl_start_potential
		else:
			#print("CRAWL STOP %+0.1f %+0.1f" % (self.crawl_ignored_potential, flag_running_progress))
			flag_running_progress -= self.crawl_ignored_potential
			self.crawl_start_potential = None

		return flag_running_progress + self.potential_leak()*100

