"""This file implements the gym environment of example PyBullet simulation.

"""

import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import math
import time
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import pybullet
from pybullet_utils import bullet_client as bc

from pybullet_envs.prediction import boxstack_pybullet_sim

import os
import pybullet_data

from pkg_resources import parse_version


class PyBulletSimGymEnv(gym.Env):
  """The gym environment to run pybullet simulations.


  """
  metadata = {"render.modes": ["human", "rgb_array"], "video.frames_per_second": 50}

  def __init__(self,
               pybullet_sim_factory=boxstack_pybullet_sim,
               render=True,
               render_sleep=False,
               debug_visualization=True,
               hard_reset=False,
               render_width=240,
               render_height=240,
               action_repeat=1,
               time_step=1. / 240.,
               num_bullet_solver_iterations=50,
               urdf_root=pybullet_data.getDataPath()):
    """Initialize the gym environment.

    Args:
      urdf_root: The path to the urdf data folder.
    """
    self._pybullet_sim_factory = pybullet_sim_factory
    self._time_step = time_step
    self._urdf_root = urdf_root
    self._observation = []
    self._action_repeat = action_repeat
    self._num_bullet_solver_iterations = num_bullet_solver_iterations
    self._env_step_counter = 0
    self._is_render = render
    self._debug_visualization = debug_visualization
    self._render_sleep = render_sleep
    self._render_width = render_width
    self._render_height = render_height
    self._cam_dist = .3
    self._cam_yaw = 50
    self._cam_pitch = -35
    self._hard_reset = True
    self._last_frame_time = 0.0

    optionstring = '--width={} --height={}'.format(render_width, render_height)

    print("urdf_root=" + self._urdf_root)

    if self._is_render:
      self._pybullet_client = bc.BulletClient(connection_mode=pybullet.GUI,
                                                         options=optionstring)
    else:
      self._pybullet_client = bc.BulletClient()

    if (debug_visualization == False):
      self._pybullet_client.configureDebugVisualizer(flag=self._pybullet_client.COV_ENABLE_GUI,
                                                     enable=0)
      self._pybullet_client.configureDebugVisualizer(
          flag=self._pybullet_client.COV_ENABLE_RGB_BUFFER_PREVIEW, enable=0)
      self._pybullet_client.configureDebugVisualizer(
          flag=self._pybullet_client.COV_ENABLE_DEPTH_BUFFER_PREVIEW, enable=0)
      self._pybullet_client.configureDebugVisualizer(
          flag=self._pybullet_client.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, enable=0)

    self._pybullet_client.setAdditionalSearchPath(urdf_root)
    self.seed()
    self.reset()

    observation_high = (self._example_sim.GetObservationUpperBound())
    observation_low = (self._example_sim.GetObservationLowerBound())

    action_dim = self._example_sim.GetActionDimension()
    self._action_bound = 1
    action_high = np.array([self._action_bound] * action_dim)
    self.action_space = spaces.Box(-action_high, action_high)
    self.observation_space = spaces.Box(observation_low, observation_high)

    self.viewer = None
    self._hard_reset = hard_reset  # This assignment need to be after reset()

  def configure(self, args):
    self._args = args

  def reset(self):
    if self._hard_reset:
      self._pybullet_client.resetSimulation()

      self._pybullet_client.setPhysicsEngineParameter(
          numSolverIterations=int(self._num_bullet_solver_iterations))
      self._pybullet_client.setTimeStep(self._time_step)

      self._example_sim = self._pybullet_sim_factory.CreateSim(
          pybullet_client=self._pybullet_client,
          urdf_root=self._urdf_root,
          time_step=self._time_step)
    else:
      self._example_sim.Reset(reload_urdf=False)

    self._env_step_counter = 0

    #self._pybullet_client.resetDebugVisualizerCamera(
    #    self._cam_dist, self._cam_yaw, self._cam_pitch, [0, 0, 0])

    return self._get_observation()

  def seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]

  def step(self, action):
    """Step forward the simulation, given the action.

    Args:
      action: the predicted state

    Returns:
      observations: The actual state.
      reward: The reward for how well the prediction matches the actual state.
      done: Whether the episode has ended.
      info: A dictionary that stores diagnostic information.

    Raises:
      ValueError: The action dimension is not the same as the number of motors.
      ValueError: The magnitude of actions is out of bounds.
    """
    if self._render_sleep:
      # Sleep, otherwise the computation takes less time than real time,
      # which will make the visualization like a fast-forward video.
      time_spent = time.time() - self._last_frame_time
      self._last_frame_time = time.time()
      time_to_sleep = self._action_repeat * self._time_step - time_spent
      if time_to_sleep > 0:
        time.sleep(time_to_sleep)

      #base_pos = self.minitaur.GetBasePosition()
      #self._pybullet_client.resetDebugVisualizerCamera(
      #    self._cam_dist, self._cam_yaw, self._cam_pitch, base_pos)

    for _ in range(self._action_repeat):
      self._example_sim.ApplyAction(action)
      self._pybullet_client.stepSimulation()

    self._env_step_counter += 1
    reward = self._reward()
    done = self._termination()
    return np.array(self._get_observation()), reward, done, {}

  def render(self, mode="rgb_array", close=False):
    if mode != "rgb_array":
      return np.array([])
    base_pos = [0, 0, 0]
    view_matrix = self._pybullet_client.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=base_pos,
        distance=self._cam_dist,
        yaw=self._cam_yaw,
        pitch=self._cam_pitch,
        roll=0,
        upAxisIndex=2)
    proj_matrix = self._pybullet_client.computeProjectionMatrixFOV(
        fov=60, aspect=float(self._render_width) / self._render_width, nearVal=0.01, farVal=100.0)
    proj_matrix = [
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0000200271606445, -1.0, 0.0, 0.0,
        -0.02000020071864128, 0.0
    ]
    (_, _, px, _, _) = self._pybullet_client.getCameraImage(
        width=self._render_width,
        height=self._render_height,
        viewMatrix=view_matrix,
        projectionMatrix=proj_matrix,
        renderer=pybullet.ER_BULLET_HARDWARE_OPENGL)  #ER_TINY_RENDERER)
    rgb_array = np.array(px, dtype=np.uint8)
    rgb_array = np.reshape(rgb_array, (self._render_height, self._render_width, 4))
    rgb_array = rgb_array[:, :, :3]
    return rgb_array

  def _termination(self):
    terminate = self._example_sim.Termination()
    return terminate

  def _reward(self):
    reward = 0
    return reward

  def _get_observation(self):
    self._observation = self._example_sim.GetObservation()
    return self._observation

  if parse_version(gym.__version__) < parse_version('0.9.6'):
    _render = render
    _reset = reset
    _seed = seed
    _step = step
