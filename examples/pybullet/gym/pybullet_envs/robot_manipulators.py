from robot_bases import MJCFBasedRobot
import numpy as np


class Reacher(MJCFBasedRobot):
	TARG_LIMIT = 0.27

	def __init__(self):
		MJCFBasedRobot.__init__(self, 'reacher.xml', 'body0', action_dim=2, obs_dim=9)

	def robot_specific_reset(self, bullet_client):
		self.jdict["target_x"].reset_current_position(
			self.np_random.uniform(low=-self.TARG_LIMIT, high=self.TARG_LIMIT), 0)
		self.jdict["target_y"].reset_current_position(
			self.np_random.uniform(low=-self.TARG_LIMIT, high=self.TARG_LIMIT), 0)
		self.fingertip = self.parts["fingertip"]
		self.target = self.parts["target"]
		self.central_joint = self.jdict["joint0"]
		self.elbow_joint = self.jdict["joint1"]
		self.central_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
		self.elbow_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)

	def apply_action(self, a):
		assert (np.isfinite(a).all())
		self.central_joint.set_motor_torque(0.05 * float(np.clip(a[0], -1, +1)))
		self.elbow_joint.set_motor_torque(0.05 * float(np.clip(a[1], -1, +1)))

	def calc_state(self):
		theta, self.theta_dot = self.central_joint.current_relative_position()
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


class Pusher(MJCFBasedRobot):
	min_target_placement_radius = 0.5
	max_target_placement_radius = 0.8
	min_object_to_target_distance = 0.1
	max_object_to_target_distance = 0.4

	def __init__(self):
		MJCFBasedRobot.__init__(self, 'pusher.xml', 'body0', action_dim=7, obs_dim=55)

	def robot_specific_reset(self, bullet_client):
		# parts
		self.fingertip = self.parts["fingertip"]
		self.target = self.parts["target"]
		self.object = self.parts["object"]

		# joints
		self.shoulder_pan_joint = self.jdict["shoulder_pan_joint"]
		self.shoulder_lift_joint = self.jdict["shoulder_lift_joint"]
		self.upper_arm_roll_joint = self.jdict["upper_arm_roll_joint"]
		self.elbow_flex_joint = self.jdict["elbow_flex_joint"]
		self.forearm_roll_joint = self.jdict["forearm_roll_joint"]
		self.wrist_flex_joint = self.jdict["wrist_flex_joint"]
		self.wrist_roll_joint = self.jdict["wrist_roll_joint"]

		self.target_pos = np.concatenate([
			self.np_random.uniform(low=-1, high=1, size=1),
			self.np_random.uniform(low=-1, high=1, size=1)
		])

		# make length of vector between min and max_target_placement_radius
		self.target_pos = self.target_pos \
						  / np.linalg.norm(self.target_pos) \
						  * self.np_random.uniform(low=self.min_target_placement_radius,
												   high=self.max_target_placement_radius, size=1)

		self.object_pos = np.concatenate([
			self.np_random.uniform(low=-1, high=1, size=1),
			self.np_random.uniform(low=-1, high=1, size=1)
		])

		# make length of vector between min and max_object_to_target_distance
		self.object_pos = self.object_pos \
						  / np.linalg.norm(self.object_pos - self.target_pos) \
						  * self.np_random.uniform(low=self.min_object_to_target_distance,
												   high=self.max_object_to_target_distance, size=1)

		# set position of objects
		self.zero_offset = np.array([0.45, 0.55])
		self.jdict["target_x"].reset_current_position(self.target_pos[0] - self.zero_offset[0], 0)
		self.jdict["target_y"].reset_current_position(self.target_pos[1] - self.zero_offset[1], 0)
		self.jdict["object_x"].reset_current_position(self.object_pos[0] - self.zero_offset[0], 0)
		self.jdict["object_y"].reset_current_position(self.object_pos[1] - self.zero_offset[1], 0)

		# randomize all joints TODO: Will this work or do we have to constrain this resetting in some way?
		self.shoulder_pan_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
		self.shoulder_lift_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
		self.upper_arm_roll_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
		self.elbow_flex_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
		self.forearm_roll_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
		self.wrist_flex_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
		self.wrist_roll_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)

	def apply_action(self, a):
		assert (np.isfinite(a).all())
		self.shoulder_pan_joint.set_motor_torque(0.05 * float(np.clip(a[0], -1, +1)))
		self.shoulder_lift_joint.set_motor_torque(0.05 * float(np.clip(a[1], -1, +1)))
		self.upper_arm_roll_joint.set_motor_torque(0.05 * float(np.clip(a[2], -1, +1)))
		self.elbow_flex_joint.set_motor_torque(0.05 * float(np.clip(a[3], -1, +1)))
		self.forearm_roll_joint.set_motor_torque(0.05 * float(np.clip(a[4], -1, +1)))
		self.wrist_flex_joint.set_motor_torque(0.05 * float(np.clip(a[5], -1, +1)))
		self.wrist_roll_joint.set_motor_torque(0.05 * float(np.clip(a[6], -1, +1)))

	def calc_state(self):
		self.to_target_vec = self.target_pos - self.object_pos
		return np.concatenate([
			np.array([j.current_position() for j in self.ordered_joints]).flatten(),  # all positions
			np.array([j.current_relative_position() for j in self.ordered_joints]).flatten(),  # all speeds
			self.to_target_vec,
			self.fingertip.pose().xyz(),
			self.object.pose().xyz(),
			self.target.pose().xyz(),
		])


class Striker(MJCFBasedRobot):
	min_target_placement_radius = 0.1
	max_target_placement_radius = 0.8
	min_object_placement_radius = 0.1
	max_object_placement_radius = 0.8

	def __init__(self):
		MJCFBasedRobot.__init__(self, 'striker.xml', 'body0', action_dim=7, obs_dim=55)

	def robot_specific_reset(self, bullet_client):
		# parts
		self.fingertip = self.parts["fingertip"]
		self.target = self.parts["target"]
		self.object = self.parts["object"]

		# joints
		self.shoulder_pan_joint = self.jdict["shoulder_pan_joint"]
		self.shoulder_lift_joint = self.jdict["shoulder_lift_joint"]
		self.upper_arm_roll_joint = self.jdict["upper_arm_roll_joint"]
		self.elbow_flex_joint = self.jdict["elbow_flex_joint"]
		self.forearm_roll_joint = self.jdict["forearm_roll_joint"]
		self.wrist_flex_joint = self.jdict["wrist_flex_joint"]
		self.wrist_roll_joint = self.jdict["wrist_roll_joint"]

		self._min_strike_dist = np.inf
		self._striked = False
		self._strike_pos = None

		# reset position and speed of manipulator
		# TODO: Will this work or do we have to constrain this resetting in some way?
		self.shoulder_pan_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
		self.shoulder_lift_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
		self.upper_arm_roll_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
		self.elbow_flex_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
		self.forearm_roll_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
		self.wrist_flex_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
		self.wrist_roll_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)

		self.zero_offset = np.array([0.45, 0.55, 0])
		self.object_pos = np.concatenate([
			self.np_random.uniform(low=-1, high=1, size=1),
			self.np_random.uniform(low=-1, high=1, size=1)
		])

		# make length of vector between min and max_object_placement_radius
		self.object_pos = self.object_pos \
						  / np.linalg.norm(self.object_pos) \
						  * self.np_random.uniform(low=self.min_object_placement_radius,
												   high=self.max_object_placement_radius, size=1)

		# reset object position
		self.jdict["object_x"].reset_current_position(self.object_pos[0] - self.zero_offset[0], 0)
		self.jdict["object_y"].reset_current_position(self.object_pos[1] - self.zero_offset[1], 0)

		self.target_pos = np.concatenate([
			self.np_random.uniform(low=-1, high=1, size=1),
			self.np_random.uniform(low=-1, high=1, size=1),
			self.np_random.uniform(low=-1, high=1, size=1)
		])

		# make length of vector between min and max_target_placement_radius
		self.target_pos = self.target_pos \
						  / np.linalg.norm(self.target_pos) \
						  * self.np_random.uniform(low=self.min_target_placement_radius,
												   high=self.max_target_placement_radius, size=1)

		self.target.reset_pose(self.target_pos - self.zero_offset, np.array([0, 0, 0, 1]))

	def apply_action(self, a):
		assert (np.isfinite(a).all())
		self.shoulder_pan_joint.set_motor_torque(0.05 * float(np.clip(a[0], -1, +1)))
		self.shoulder_lift_joint.set_motor_torque(0.05 * float(np.clip(a[1], -1, +1)))
		self.upper_arm_roll_joint.set_motor_torque(0.05 * float(np.clip(a[2], -1, +1)))
		self.elbow_flex_joint.set_motor_torque(0.05 * float(np.clip(a[3], -1, +1)))
		self.forearm_roll_joint.set_motor_torque(0.05 * float(np.clip(a[4], -1, +1)))
		self.wrist_flex_joint.set_motor_torque(0.05 * float(np.clip(a[5], -1, +1)))
		self.wrist_roll_joint.set_motor_torque(0.05 * float(np.clip(a[6], -1, +1)))

	def calc_state(self):
		self.to_target_vec = self.target_pos - self.object_pos
		return np.concatenate([
			np.array([j.current_position() for j in self.ordered_joints]).flatten(),  # all positions
			np.array([j.current_relative_position() for j in self.ordered_joints]).flatten(),  # all speeds
			self.to_target_vec,
			self.fingertip.pose().xyz(),
			self.object.pose().xyz(),
			self.target.pose().xyz(),
		])


class Thrower(MJCFBasedRobot):
	min_target_placement_radius = 0.1
	max_target_placement_radius = 0.8
	min_object_placement_radius = 0.1
	max_object_placement_radius = 0.8

	def __init__(self):
		MJCFBasedRobot.__init__(self, 'thrower.xml', 'body0', action_dim=7, obs_dim=48)

	def robot_specific_reset(self, bullet_client):
		# parts
		self.fingertip = self.parts["fingertip"]
		self.target = self.parts["target"]
		self.object = self.parts["object"]

		# joints
		self.shoulder_pan_joint = self.jdict["shoulder_pan_joint"]
		self.shoulder_lift_joint = self.jdict["shoulder_lift_joint"]
		self.upper_arm_roll_joint = self.jdict["upper_arm_roll_joint"]
		self.elbow_flex_joint = self.jdict["elbow_flex_joint"]
		self.forearm_roll_joint = self.jdict["forearm_roll_joint"]
		self.wrist_flex_joint = self.jdict["wrist_flex_joint"]
		self.wrist_roll_joint = self.jdict["wrist_roll_joint"]

		self._object_hit_ground = False
		self._object_hit_location = None

		# reset position and speed of manipulator
		# TODO: Will this work or do we have to constrain this resetting in some way?
		self.shoulder_pan_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
		self.shoulder_lift_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
		self.upper_arm_roll_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
		self.elbow_flex_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
		self.forearm_roll_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
		self.wrist_flex_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
		self.wrist_roll_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)

		self.zero_offset = np.array([0.45, 0.55, 0])
		self.object_pos = np.concatenate([
			self.np_random.uniform(low=-1, high=1, size=1),
			self.np_random.uniform(low=-1, high=1, size=1),
			self.np_random.uniform(low=-1, high=1, size=1)
		])

		# make length of vector between min and max_object_placement_radius
		self.object_pos = self.object_pos \
						  / np.linalg.norm(self.object_pos) \
						  * self.np_random.uniform(low=self.min_object_placement_radius,
												   high=self.max_object_placement_radius, size=1)

		# reset object position
		self.parts["object"].reset_pose(self.object_pos - self.zero_offset, np.array([0, 0, 0, 1]))

		self.target_pos = np.concatenate([
			self.np_random.uniform(low=-1, high=1, size=1),
			self.np_random.uniform(low=-1, high=1, size=1),
			self.np_random.uniform(low=-1, high=1, size=1)
		])

		# make length of vector between min and max_target_placement_radius
		self.target_pos = self.target_pos \
						  / np.linalg.norm(self.target_pos) \
						  * self.np_random.uniform(low=self.min_target_placement_radius,
												   high=self.max_target_placement_radius, size=1)

		self.parts["target"].reset_pose(self.target_pos - self.zero_offset, np.array([0, 0, 0, 1]))

	def apply_action(self, a):
		assert (np.isfinite(a).all())
		self.shoulder_pan_joint.set_motor_torque(0.05 * float(np.clip(a[0], -1, +1)))
		self.shoulder_lift_joint.set_motor_torque(0.05 * float(np.clip(a[1], -1, +1)))
		self.upper_arm_roll_joint.set_motor_torque(0.05 * float(np.clip(a[2], -1, +1)))
		self.elbow_flex_joint.set_motor_torque(0.05 * float(np.clip(a[3], -1, +1)))
		self.forearm_roll_joint.set_motor_torque(0.05 * float(np.clip(a[4], -1, +1)))
		self.wrist_flex_joint.set_motor_torque(0.05 * float(np.clip(a[5], -1, +1)))
		self.wrist_roll_joint.set_motor_torque(0.05 * float(np.clip(a[6], -1, +1)))

	def calc_state(self):
		self.to_target_vec = self.target_pos - self.object_pos
		return np.concatenate([
			np.array([j.current_position() for j in self.ordered_joints]).flatten(),  # all positions
			np.array([j.current_relative_position() for j in self.ordered_joints]).flatten(),  # all speeds
			self.to_target_vec,
			self.fingertip.pose().xyz(),
			self.object.pose().xyz(),
			self.target.pose().xyz(),
		])
