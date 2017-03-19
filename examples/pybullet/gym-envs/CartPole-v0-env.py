#!/usr/bin/env python

# A pole is attached by an un-actuated joint to a cart, which moves along a frictionless track. 
# The system is controlled by applying a force of +1 or -1 to the cart. The pendulum starts upright, and the goal is to prevent it from falling over. 
# A reward of +1 is provided for every timestep that the pole remains upright. 
# The episode ends when the pole is more than 15 degrees from vertical, or the cart moves more than 2.4 units from the center.
#
# CartPole-v0 defines "solving" as getting average reward of 195.0 over 100 consecutive trials.
#
# This environment is based on the OpenAI CartPole-v0 implementation of the cart-pole problem described by Barto, Sutton, and Anderson [Barto83].

from collections import *
import gym
from gym import spaces
import numpy as np
import pybullet
import sys
import time

np.set_printoptions(precision=3, suppress=True, linewidth=10000)

def add_opts(parser):
	parser.add_argument('--gui', action='store_true')
	parser.add_argument('--initial-force', type=float, default=55.0,
                      help="magnitude of initial push, in random direction")
	parser.add_argument('--no-random-theta', action='store_true')
	parser.add_argument('--action-repeats', type=int, default=2,
                      help="number of action repeats")
	parser.add_argument('--steps-per-repeat', type=int, default=5,
                      help="number of sim steps per repeat")
	parser.add_argument('--max-episode-len', type=int, default=200,
                      help="maximum episode len for cartpole")
	parser.add_argument('--reward-calc', type=str, default='fixed',
                      help="'fixed': 1 per step. 'angle': 2*max_angle - ox - oy. 'action': 1.5 - |action|. 'angle_action': both angle and action")

class CartPolev0(gym.Env):
	
	def __init__(sef, opts, discrete_actions):
		self.gui = opts.gui
		self.max_episode_len = opts.max_episode_len
		
		# threshold for pole position.
		# if absolute x or y moves outside this we finish episode
		self.pos_threshold = 2.4  # TODO: higher?

		# threshold for angle from z-axis.
		# if x or y > this value we finish episode.
		self.angle_threshold = 0.26  # radians; ~= 15deg

		# initial push force. this should be enough that taking no action will always
		# result in pole falling after initial_force_steps but not so much that you
		# can't recover. see also initial_force_steps.
		self.initial_force = opts.initial_force
		
		# number of sim steps initial force is applied.
		# (see initial_force)
		self.initial_force_steps = 30
		
		# whether we do initial push in a random direction
		# if false we always push with along x-axis (simplee problem, useful for debugging)
		self.random_theta = not opts.no_random_theta

		# true if action space is discrete; 5 values; no push, left, right, up & down
		# false if action space is continuous; fx, fy both (-action_force, action_force)
		self.discrete_actions = discrete_actions
		
		# 5 discrete actions: no push, left, right, up, down
		# 2 continuous action elements; fx & fy
		if self.discrete_actions:
			self.action_space = spaces.Discrete(5)
		else:
			self.action_space = spaces.Box(-1.0, 1.0, shape=(1, 2))
			
		# how many time to repeat each action per step().
		# and how many sim steps to do per state capture
		# (total number of sim steps = action_repeats * steps_per_repeat
		self.repeats = opts.action_repeats
		self.steps_per_repeat = opts.steps_per_repeat
		
		# in the low dimensional case obs space for problem is (R, 2, 7)
		#  R = number of repeats
		#  2 = two items; cart & pole
		#  7d tuple for pos + orientation pose
		state_shape = (self.repeats, 2, 7)
		
		# Our observations can be within this box
		float_max = np.finfo(np.float32).max
		self.observation_space = gym.spaces.Box(-float_max, float_max, state_shape)
		
		# check reward type
		assert opts.reward_calc in ['fixed', 'angle', 'action', 'angle_action']
		self.reward_calc = opts.reward_calc
		
		# setup bullet
		p.connect(p.GUI if self.gui else p.DIRECT)
		p.setGravity(0, 0, -9.81)
		p.loadURDF("models/ground.urdf", 0,0,0, 0,0,0,1)
		self.cart = p.loadURDF("models/cart.urdf", 0,0,0.08, 0,0,0,1)
		self.pole = p.loadURDF("models/pole.urdf", 0,0,0.35, 0,0,0,1)
		
	def _configure(self, display=None):
		pass

	def _seed(self, seed=None):
		pass

	def _render(self, mode, close):
		pass
		
	def _step(self, action):
		if self.done:
		  print >>sys.stderr, "Why is step called when the episode is done?"
		  return np.copy(self.state), 0, True, {}

		info = {}

		# based on action decide the x and y forces
		fx = fy = 0
		if self.discrete_actions:
		  if action == 0:
			pass
		  elif action == 1:
			fx = self.action_force
		  elif action == 2:
			fx = -self.action_force
		  elif action == 3:
			fy = self.action_force
		  elif action == 4:
			fy = -self.action_force
		  else:
			raise Exception("unknown discrete action [%s]" % action)
		else: # continuous actions
		  fx, fy = action[0] * self.action_force

		# step simulation forward. at the end of each repeat we set part of the step's
		# state by capture the cart & pole state in some form.
		for r in xrange(self.repeats):
		  for _ in xrange(self.steps_per_repeat):
			p.stepSimulation()
			p.applyExternalForce(self.cart, -1, (fx,fy,0), (0,0,0), p.WORLD_FRAME)
			if self.delay > 0:
			  time.sleep(self.delay)
		  self.set_state_element_for_repeat(r)
		self.steps += 1

		# Check for out of bounds by position or orientation on pole.
		# we (re)fetch pose explicitly rather than depending on fields in state.
		(x, y, _z), orient = p.getBasePositionAndOrientation(self.pole)
		ox, oy, _oz = p.getEulerFromQuaternion(orient)  # roll / pitch / yaw
		if abs(x) > self.pos_threshold or abs(y) > self.pos_threshold:
		  info['done_reason'] = 'out of position bounds'
		  self.done = True
		  reward = 0.0
		elif abs(ox) > self.angle_threshold or abs(oy) > self.angle_threshold:
		  # TODO: probably better to do explicit angle from z?
		  info['done_reason'] = 'out of orientation bounds'
		  self.done = True
		  reward = 0.0
		# check for end of episode (by length)
		if self.steps >= self.max_episode_len:
		  info['done_reason'] = 'episode length'
		  self.done = True

		# calc reward, fixed base of 1.0
		reward = 1.0
		if self.reward_calc == "angle" or self.reward_calc == "angle_action":
		  # clip to zero since angles can be past threshold
		  reward += max(0, 2 * self.angle_threshold - np.abs(ox) - np.abs(oy))
		if self.reward_calc == "action" or self.reward_calc == "angle_action":
		  # max norm will be sqr(2) ~= 1.4.
		  # reward is already 1.0 to add another 0.5 as o0.1 buffer from zero
		  reward += 0.5 - np.linalg.norm(action[0])

		# log this event.
		# TODO in the --use-raw-pixels case would be nice to have poses in state repeats too.
		if self.event_log:
		  self.event_log.add(self.state, action, reward)

		# return observation
		return np.copy(self.state), reward, self.done, info
		
	def set_state_element_for_repeat(self, repeat):
		# in low dim case state is (R, 2, 7)
		# R -> repeat, 2 -> 2 objects (cart & pole), 7 -> 7d pose
		self.state[repeat][0] = state_fields_of_pose_of(self.cart)
		self.state[repeat][1] = state_fields_of_pose_of(self.pole)
		
	def _reset(self):
		# reset state
		self.steps = 0
		self.done = False

		# reset pole on cart in starting poses
		p.resetBasePositionAndOrientation(self.cart, (0,0,0.08), (0,0,0,1))
		p.resetBasePositionAndOrientation(self.pole, (0,0,0.35), (0,0,0,1))
		for _ in xrange(100): p.stepSimulation()

		# give a fixed force push in a random direction to get things going...
		theta = (np.random.random() * 2 * np.pi) if self.random_theta else 0.0
		fx, fy = self.initial_force * np.cos(theta), self.initial_force * np.sin(theta)
		for _ in xrange(self.initial_force_steps):
		  p.stepSimulation()
		  p.applyExternalForce(self.cart, -1, (fx, fy, 0), (0, 0, 0), p.WORLD_FRAME)
		  if self.delay > 0:
			time.sleep(self.delay)

		# bootstrap state by running for all repeats
		for i in xrange(self.repeats):
		  self.set_state_element_for_repeat(i)

		# return this state
		return np.copy(self.state)

#<mujoco model="cart-pole">
#    <compiler inertiafromgeom="true"/>
#    <default>
#        <joint limited='true' damping='1' armature='0'  />
#        <geom contype="0" friction="1 0.1 0.1" rgba="0.7 0.7 0 1" />
#        <tendon />
#        <motor ctrlrange="-3 3"/>
#    </default>
#    <option	timestep="0.02" gravity="0 0 -9.81" />
#    <size nstack="3000"/>
#    <worldbody>
#        <!--geom name="ground" type="plane" pos="0 0 0" /-->
#        <geom name="rail" type="capsule" pos="0 0 0" quat="0.707 0 0.707 0" size="0.02 1" rgba="0.3 0.3 0.7 1" />
#        <body name="cart" pos="0 0 0">
#            <joint name="slider" type="slide" limited="true" pos="0 0 0" axis="1 0 0" range="-1 1" />
#            <geom name="cart" type="capsule" pos="0 0 0" quat="0.707 0 0.707 0" size="0.1 0.1" />
#            <body name="pole" pos="0 0 0">
#                <joint name="hinge" type="hinge" pos="0 0 0" axis="0 1 0" range='-90 90'/>
#                <geom name="cpole" type="capsule" fromto="0 0 0 0.001 0 0.6" size="0.049 0.3" rgba="0 0.7 0.7 1" />
#                <!--                 
#				<body name="pole2" pos="0.001 0 0.6"><joint name="hinge2" type="hinge" pos="0 0 0" axis="0 1 0"/>
#					<geom name="cpole2" type="capsule" fromto="0 0 0 0 0 0.6" size="0.05 0.3" rgba="0.7 0 0.7 1"/>
#					<site name="tip2" pos="0 0 .6"/>
#				</body>    -->
#            </body>
#        </body>
#    </worldbody>
#    <actuator>
#        <motor name="slide" joint="slider" gear="100"/>
#        <!-- <general name="slide" 	dyntype="filter" dynprm="0.1" trnprm="100 0 0 0 0" target="slider" /> -->
#    </actuator>
#    <tendon>
#        <fixed name="dummy0">
#            <joint joint="slider"  coef="1"/>
#        </fixed>
#        <fixed name="dummy1">
#            <joint joint="hinge"  coef="1"/>
#        </fixed>
#    </tendon>
#</mujoco>