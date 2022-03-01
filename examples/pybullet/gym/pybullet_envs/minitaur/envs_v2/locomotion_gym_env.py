# Lint as: python3
"""This file implements the locomotion gym env."""

import atexit
import collections
import time
from typing import Any, Callable, Sequence, Text, Union
import gin
import gym
import numpy as np

from pybullet_envs.minitaur.envs_v2 import base_client
from pybullet_utils import bullet_client
import pybullet_data
import pybullet
from pybullet_envs.minitaur.envs import minitaur_logging
from pybullet_envs.minitaur.envs import minitaur_logging_pb2
#from pybullet_envs.minitaur.envs import minitaur_logging
#from pybullet_envs.minitaur.envs import minitaur_logging_pb2
from pybullet_envs.minitaur.envs_v2.evaluation import metric_logger
from pybullet_envs.minitaur.envs_v2.scenes import scene_base
from pybullet_envs.minitaur.envs_v2.sensors import sensor
from pybullet_envs.minitaur.envs_v2.sensors import space_utils
from pybullet_envs.minitaur.envs_v2.utilities import rendering_utils
from pybullet_envs.minitaur.robots import autonomous_object
from pybullet_envs.minitaur.robots import robot_base

_ACTION_EPS = 0.01
_NUM_SIMULATION_ITERATION_STEPS = 300
_LOG_BUFFER_LENGTH = 5000

SIM_CLOCK = 'SIM_CLOCK'

# Exports this symbol so we can use it in the config file.
gin.constant('locomotion_gym_env.SIM_CLOCK', SIM_CLOCK)

# This allows us to bind @time.time in the gin configuration.
gin.external_configurable(time.time, module='time')


# TODO(b/122048194): Enable position/torque/hybrid control mode.
@gin.configurable
class LocomotionGymEnv(gym.Env):
  """The gym environment for the locomotion tasks."""
  metadata = {
      'render.modes': ['human', 'rgb_array', 'topdown'],
      'video.frames_per_second': 100
  }

  def __init__(self,
               gym_config,
               clock: Union[Callable[..., float], Text] = 'SIM_CLOCK',
               robot_class: Any = None,
               scene: scene_base.SceneBase = None,
               sensors: Sequence[sensor.Sensor] = None,
               task: Any = None,
               env_randomizers: Any = None):
    """Initializes the locomotion gym environment.

    Args:
      gym_config: An instance of LocomotionGymConfig.
      clock: The clock source to be used for the gym env. The clock should
        return a timestamp in seconds. Setting clock == "SIM_CLOCK" will enable
        the built-in simulation clock. For real robot experiments, we can use
        time.time or other clock wall clock sources.
      robot_class: A class of a robot. We provide a class rather than an
        instance due to hard_reset functionality. Parameters are expected to be
        configured with gin.
      scene: An object for managing the robot's surroundings.
      sensors: A list of environmental sensors for observation.
      task: A callable function/class to calculate the reward and termination
        condition. Takes the gym env as the argument when calling.
      env_randomizers: A list of EnvRandomizer(s). An EnvRandomizer may
        randomize the physical property of minitaur, change the terrrain during
        reset(), or add perturbation forces during step().
      client_factory: A function to create a simulation client, it can be a
        pybullet client.

    Raises:
      ValueError: If the num_action_repeat is less than 1.

    """
    self._pybullet_client = None
    self._metric_logger = metric_logger.MetricLogger()
    # TODO(sehoonha) split observation and full-state sensors (b/129858214)

    # Makes sure that close() is always called to flush out the logs to the
    # disk.
    atexit.register(self.close)
    self.seed()
    self._gym_config = gym_config
    if robot_class is None:
      raise ValueError('robot_class cannot be None.')
    self._robot_class = robot_class
    if issubclass(self._robot_class, robot_base.RobotBase):
      self._use_new_robot_class = True
    else:
      self._use_new_robot_class = False
    self._robot = None

    self._scene = scene or scene_base.SceneBase()

    # TODO(sehoonha) change the data structure to dictionary
    self._env_sensors = list(sensors) if sensors is not None else list()

    # TODO(b/152161457): Make logging a standalone module.
    self._log_path = gym_config.log_path
    self._logging = minitaur_logging.MinitaurLogging(self._log_path)
    self._episode_proto = minitaur_logging_pb2.MinitaurEpisode()
    self._data_dir = gym_config.data_dir

    self._task = task

    self._env_randomizers = env_randomizers if env_randomizers else []

    # Simulation related parameters.
    self._num_action_repeat = gym_config.simulation_parameters.num_action_repeat
    self._on_rack = gym_config.simulation_parameters.robot_on_rack
    if self._num_action_repeat < 1:
      raise ValueError('number of action repeats should be at least 1.')
    # TODO(b/73829334): Fix the value of self._num_bullet_solver_iterations.
    self._num_bullet_solver_iterations = int(_NUM_SIMULATION_ITERATION_STEPS /
                                             self._num_action_repeat)

    self._sim_time_step = gym_config.simulation_parameters.sim_time_step_s
    # The sim step counter is an internal varialbe to count the number of
    # pybullet stepSimulation() has been called since last reset.
    self._sim_step_counter = 0

    self._env_time_step = self._num_action_repeat * self._sim_time_step
    # The env step counter accounts for how many times env.step has been
    # called since reset.
    self._env_step_counter = 0

    if clock == SIM_CLOCK:
      self._clock = self._get_sim_time
    else:
      self._clock = clock

    # Creates the bullet client.
    self._is_render = gym_config.simulation_parameters.enable_rendering
    # The wall-clock time at which the last frame is rendered.
    self._last_frame_time = 0.0
    
    if gym_config.simulation_parameters.enable_rendering:
      self._pybullet_client = bullet_client.BulletClient(connection_mode=pybullet.GUI)
      self._pybullet_client.configureDebugVisualizer(
        pybullet.COV_ENABLE_GUI,
        gym_config.simulation_parameters.enable_rendering_gui)
    else:
      self._pybullet_client = bullet_client.BulletClient()
    if gym_config.simulation_parameters.egl_rendering:
      self._pybullet_client.loadPlugin('eglRendererPlugin')

    self._pybullet_client.setAdditionalSearchPath(
      pybullet_data.getDataPath())

    # If enabled, save the performance profile to profiling_path
    # Use Google Chrome about://tracing to open the file
    if gym_config.profiling_path is not None:
      self._profiling_slot = self._pybullet_client.startStateLogging(
          self._pybullet_client.STATE_LOGGING_PROFILE_TIMINGS,
          gym_config.profiling_path)
      self._profiling_counter = 10
    else:
      self._profiling_slot = -1

    # Set the default render options. TODO(b/152161124): Make rendering a
    # standalone module.
    self._camera_target = gym_config.simulation_parameters.camera_target
    self._camera_dist = gym_config.simulation_parameters.camera_distance
    self._camera_yaw = gym_config.simulation_parameters.camera_yaw
    self._camera_pitch = gym_config.simulation_parameters.camera_pitch
    self._render_width = gym_config.simulation_parameters.render_width
    self._render_height = gym_config.simulation_parameters.render_height

    # Loads the environment and robot. Actions space will be created as well.
    self._hard_reset = True
    self._observation_dict = {}
    self.reset()
    self._hard_reset = gym_config.simulation_parameters.enable_hard_reset

    # Construct the observation space from the list of sensors.
    self.observation_space = (
        space_utils.convert_sensors_to_gym_space_dictionary([
            sensor for sensor in self.all_sensors()
            if sensor.get_name() not in self._gym_config.ignored_sensor_list
        ]))

  def __del__(self):
    self.close()

  def _load_old_robot_class(self):
    self._robot = self._robot_class(
        pybullet_client=self._pybullet_client, on_rack=self._on_rack)
    self._action_list = []
    action_upper_bound = []
    action_lower_bound = []
    for action in self._gym_config.actions:
      self._action_list.append(action.name)
      action_upper_bound.append(action.upper_bound)
      action_lower_bound.append(action.lower_bound)
    self.action_space = gym.spaces.Box(
        np.array(action_lower_bound),
        np.array(action_upper_bound),
        dtype=np.float32)

  def _load_new_robot_class(self):
    self._robot = self._robot_class(
        pybullet_client=self._pybullet_client, clock=self._clock)
    self.action_space = self._robot.action_space

  def _load(self):
    self._pybullet_client.resetSimulation()
    self._pybullet_client.setPhysicsEngineParameter(
        numSolverIterations=self._num_bullet_solver_iterations)
    self._pybullet_client.setTimeStep(self._sim_time_step)
    self._pybullet_client.setGravity(0, 0, -10)
    self._pybullet_client.setPhysicsEngineParameter(enableConeFriction=0)

    # Disable rendering during scene loading will speed up simulation.
    if self._is_render:
      self._pybullet_client.configureDebugVisualizer(
          self._pybullet_client.COV_ENABLE_RENDERING, 0)

    # Rebuild the scene.
    self._scene.build_scene(self._pybullet_client)

    # TODO(b/151975607): Deprecate old robot support.
    if self._use_new_robot_class:
      self._load_new_robot_class()
    else:
      self._load_old_robot_class()

    # Check action space.
    if (isinstance(self.action_space, gym.spaces.Box) and
        not np.all(self.action_space.low < self.action_space.high)):
      raise ValueError(f'Action space contains invalid dimensions, '
                       f'action space low = {self.action_space.low}, '
                       f'action space high = {self.action_space.high}')

    for an_object in self._dynamic_objects():
      an_object.set_clock(self._clock)

    # Enable rendering after loading finishes.
    if self._is_render:
      self._pybullet_client.configureDebugVisualizer(
          self._pybullet_client.COV_ENABLE_RENDERING, 1)

  def close(self):
    atexit.unregister(self.close)
    #if self._pybullet_client:
    
    if self._log_path is not None:
      self._logging.save_episode(self._episode_proto)
    for sensor_ in self.all_sensors():
      sensor_.on_terminate(self)
    if self._robot:
      if self._use_new_robot_class:
        self._robot.terminate()
      else:
        self._robot.Terminate()
    if self._pybullet_client:
      self._pybullet_client.disconnect()
      self._pybullet_client = None

  def seed(self, seed=None):
    self.np_random, self.np_random_seed = gym.utils.seeding.np_random(seed)
    return [self.np_random_seed]

  def _dynamic_objects(self):
    """Returns the python objects controlling moving obstacles."""
    if self._scene:
      return self._scene.dynamic_objects
    else:
      return []

  def all_sensors(self):
    """Returns all robot, environmental and dynamic objects sensors."""
    if self._use_new_robot_class:
      all_sensors = list(self._env_sensors)
      if self._robot:
        all_sensors.extend(list(self._robot.sensors))
      for obj in self._dynamic_objects():
        all_sensors.extend(obj.sensors)

      # The new way of adding task specific sensors to the sensor lists.
      if hasattr(self._task, 'sensors'):
        all_sensors.extend(self._task.sensors)
      return all_sensors
    else:
      # This is a workaround due to the issue in b/130128505#comment5
      task_sensor = ([self._task]
                     if isinstance(self._task, sensor.Sensor) else [])
      robot_sensors = []
      if self._robot:
        robot_sensors = self._robot.GetAllSensors()
      return robot_sensors + self._env_sensors + task_sensor

  def sensor_by_name(self, name):
    """Returns the sensor with the given name, or None if not exist."""
    # TODO(b/154162104): Store sensors as dictionary.
    for sensor_ in self.all_sensors():
      if sensor_.get_name() == name:
        return sensor_
    return None

  @gin.configurable('locomotion_gym_env.LocomotionGymEnv.reset')
  def reset(
      self,
      initial_motor_angles=None,
      reset_duration=1.0,
      reset_visualization_camera=True,
  ):
    """Resets the robot's position in the world or rebuild the sim world.

    The simulation world will be rebuilt if self._hard_reset is True.

    Args:
      initial_motor_angles: A list of Floats. The desired joint angles after
        reset. If None, the robot will use its built-in value.
      reset_duration: Float. The time (in seconds) needed to rotate all motors
        to the desired initial values.
      reset_visualization_camera: Whether to reset debug visualization camera on
        reset.

    Returns:
      A numpy array contains the initial observation after reset.
    """
    

    self._env_step_counter = 0
    self._sim_step_counter = 0
    self._last_reset_time = self._clock()
    self._metric_logger.reset_episode()

    # Clear the simulation world and rebuild the robot interface.
    if self._hard_reset:
      self._load()

    # Resets the scene
    self._scene.reset()

    # Resets the robot with the provided init parameters.
    if self._use_new_robot_class:
      self._robot.reset()
    else:
      self._robot.Reset(
          reload_urdf=False,
          default_motor_angles=initial_motor_angles,
          reset_time=reset_duration)

    # Flush the logs to disc and reinitialize the logging system.
    if self._log_path is not None:
      self._logging.save_episode(self._episode_proto)
      self._episode_proto = minitaur_logging_pb2.MinitaurEpisode()
      minitaur_logging.preallocate_episode_proto(self._episode_proto,
                                                 _LOG_BUFFER_LENGTH,
                                                 self._robot)

    # TODO(b/152161124): Move this part to the renderer module.
    if reset_visualization_camera:
      self._pybullet_client.resetDebugVisualizerCamera(self._camera_dist,
                                                       self._camera_yaw,
                                                       self._camera_pitch,
                                                       [0, 0, 0])

    # Create an example last action based on the type of action space.
    self._last_action = space_utils.create_constant_action(self.action_space)

    for s in self.all_sensors():
      s.on_reset(self)

    if self._task and hasattr(self._task, 'reset'):
      self._task.reset(self)

    # Loop over all env randomizers.
    for env_randomizer in self._env_randomizers:
      env_randomizer.randomize_env(self)

    for obj in self._dynamic_objects():
      obj.reset()

    # Initialize the robot base position.
    if self._use_new_robot_class:
      self._last_base_position = self._robot.base_position
    else:
      self._last_base_position = self._robot.GetBasePosition()

    # Resets the observations again, since randomizers might change the env.
    for s in self.all_sensors():
      s.on_reset(self)

    
    self._last_reset_time = self._clock()
    return self._get_observation()

  def _wait_for_rendering(self):
    # Sleep, otherwise the computation takes less time than real time,
    # which will make the visualization like a fast-forward video.
    time_spent = time.time() - self._last_frame_time
    self._last_frame_time = time.time()
    time_to_sleep = self._env_time_step - time_spent
    if time_to_sleep > 0:
      time.sleep(time_to_sleep)

    # Also keep the previous orientation of the camera set by the user.
    [yaw, pitch, dist] = self._pybullet_client.getDebugVisualizerCamera()[8:11]
    self._pybullet_client.resetDebugVisualizerCamera(dist, yaw, pitch,
                                                     self._last_base_position)

  def _step_old_robot_class(self, action):
    self._last_base_position = self._robot.GetBasePosition()
    self._last_action = action

    if self._is_render:
      self._wait_for_rendering()

    for env_randomizer in self._env_randomizers:
      env_randomizer.randomize_step(self)

    self._robot.Step(action)

    if self._profiling_slot >= 0:
      self._profiling_counter -= 1
      if self._profiling_counter == 0:
        self._pybullet_client.stopStateLogging(self._profiling_slot)

    if self._log_path is not None:
      minitaur_logging.update_episode_proto(self._episode_proto, self._robot,
                                            action, self._env_step_counter)
    reward = self._reward()

    for s in self.all_sensors():
      s.on_step(self)

    if self._task and hasattr(self._task, 'update'):
      self._task.update(self)  # TODO(b/154635313): resolve API mismatch

    done = self._termination()
    self._env_step_counter += 1
    # TODO(b/161941829): terminate removed for now, change terminate to other
    # names.
    return self._get_observation(), reward, done, {}

  def _step_new_robot_class(self, action):
    self._last_base_position = self._robot.base_position
    self._last_action = action

    if self._is_render:
      self._wait_for_rendering()

    for env_randomizer in self._env_randomizers:
      env_randomizer.randomize_step(self)

    action = self._robot.pre_control_step(action, self._env_time_step)
    for obj in self._dynamic_objects():
      obj.pre_control_step(autonomous_object.AUTONOMOUS_ACTION)
    for _ in range(self._num_action_repeat):
      for env_randomizer in self._env_randomizers:
        env_randomizer.randomize_sub_step(self, i, self._num_action_repeat)
      self._robot.apply_action(action)
      for obj in self._dynamic_objects():
        obj.update(self.get_time_since_reset(), self._observation_dict)
        obj.apply_action(autonomous_object.AUTONOMOUS_ACTION)

      self._pybullet_client.stepSimulation()
      self._sim_step_counter += 1

      self._robot.receive_observation()
      for obj in self._dynamic_objects():
        obj.receive_observation()

      for s in self.all_sensors():
        s.on_new_observation()

    self._robot.post_control_step()
    for obj in self._dynamic_objects():
      obj.post_control_step()

    if self._profiling_slot >= 0:
      self._profiling_counter -= 1
      if self._profiling_counter == 0:
        self._pybullet_client.stopStateLogging(self._profiling_slot)

    if self._log_path is not None:
      minitaur_logging.update_episode_proto(self._episode_proto, self._robot,
                                            action, self._env_step_counter)
    reward = self._reward()

    for s in self.all_sensors():
      s.on_step(self)

    if self._task and hasattr(self._task, 'update'):
      self._task.update(self)  # TODO(b/154635313): resolve API mismatch

    done = self._termination()
    self._env_step_counter += 1
    # TODO(b/161941829): terminate removed for now, change terminate to other
    # names.
    return self._get_observation(), reward, done, {}

  def step(self, action):
    """Step forward the simulation, given the action.

    Args:
      action: Can be a list of desired motor angles for all motors when the
        robot is in position control mode; A list of desired motor torques. Or a
        list of tuples (q, qdot, kp, kd, tau) for hybrid control mode. The
        action must be compatible with the robot's motor control mode. Also, we
        are not going to use the leg space (swing/extension) definition at the
        gym level, since they are specific to Minitaur.

    Returns:
      observations: The observation dictionary. The keys are the sensor names
        and the values are the sensor readings.
      reward: The reward for the current state-action pair.
      done: Whether the episode has ended.
      info: A dictionary that stores diagnostic information.

    Raises:
      ValueError: The action dimension is not the same as the number of motors.
      ValueError: The magnitude of actions is out of bounds.
    """
    # TODO(b/151975607): Finish the migration and remove old robot class
    # support.
    if self._use_new_robot_class:
      return self._step_new_robot_class(action)
    else:
      return self._step_old_robot_class(action)

  @gin.configurable('locomotion_gym_env.LocomotionGymEnv.render')
  def render(self, mode='rgb_array'):
    
    if mode == 'topdown':
      # Provide ground height if we know it. Otherwise leave it as gin
      # configurable.
      if hasattr(self.scene, 'ground_height'):
        return rendering_utils.render_topdown(
            self._pybullet_client, ground_height=self.scene.ground_height)
      else:
        return rendering_utils.render_topdown(self._pybullet_client)

    if mode != 'rgb_array':
      raise ValueError('Unsupported render mode:{}'.format(mode))

    if self._camera_target is not None:
      target_position = self._camera_target
    else:
      target_position = self._last_base_position
    view_matrix = self._pybullet_client.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=target_position,
        distance=self._camera_dist,
        yaw=self._camera_yaw,
        pitch=self._camera_pitch,
        roll=0,
        upAxisIndex=2)
    proj_matrix = self._pybullet_client.computeProjectionMatrixFOV(
        fov=60,
        aspect=float(self._render_width) / self._render_height,
        nearVal=0.1,
        farVal=100.0)
    return rendering_utils.render_image(self._pybullet_client,
                                        self._render_width, self._render_height,
                                        view_matrix, proj_matrix)

  @property
  def scene(self):
    return self._scene

  @property
  def rendering_enabled(self):
    return self._is_render

  @property
  def env_randomizers(self):
    return self._env_randomizers

  @property
  def last_base_position(self):
    return self._last_base_position

  @property
  def gym_config(self):
    return self._gym_config

  def _termination(self):
    if not self._robot.is_safe:
      return True

    if self._task and hasattr(self._task, 'done'):
      return self._task.done(self)  # TODO(b/154635313): resolve API mismatch

    return False

  def _reward(self):
    if self._task:
      return self._task.reward(self)  # TODO(b/154635313): resolve API mismatch
    return 0

  def _get_observation(self):
    """Get observation of this environment from a list of sensors.

    Returns:
      observations: dictionary of sensory observation with sensor name as key
        and corresponding observation in numpy array as value.
    """
    sensors_dict = {}
    for s in self.all_sensors():
      if s.get_name() in self._gym_config.ignored_sensor_list:
        continue

      obs = s.get_observation()
      if isinstance(obs, dict):
        sensors_dict.update(obs)
      else:
        sensors_dict[s.get_name()] = obs

    self._observation_dict = collections.OrderedDict(
        sorted(sensors_dict.items()))
    return self._observation_dict

  def set_time_step(self, num_action_repeat, sim_step=0.001):
    """Sets the time step of the environment.

    Args:
      num_action_repeat: The number of simulation steps/action repeats to be
        executed when calling env.step().
      sim_step: The simulation time step in PyBullet. By default, the simulation
        step is 0.001s, which is a good trade-off between simulation speed and
        accuracy.

    Raises:
      ValueError: If the num_action_repeat is less than 1.
    """
    if num_action_repeat < 1:
      raise ValueError('number of action repeats should be at least 1.')
    self._sim_time_step = sim_step
    self._num_action_repeat = num_action_repeat
    self._env_time_step = sim_step * num_action_repeat
    self._num_bullet_solver_iterations = int(
        _NUM_SIMULATION_ITERATION_STEPS / self._num_action_repeat)
    self._pybullet_client.setPhysicsEngineParameter(
        numSolverIterations=self._num_bullet_solver_iterations)
    self._pybullet_client.setTimeStep(self._sim_time_step)
    if not self._use_new_robot_class:
      self._robot.SetTimeSteps(self._num_action_repeat, self._sim_time_step)

  def _get_sim_time(self):
    """Returns the simulation time since the sim resets."""
    return self._sim_step_counter * self._sim_time_step

  def get_time_since_reset(self):
    """Get the time passed (in seconds) since the last reset.

    Returns:
      Time in seconds since the last reset.
    """
    if self._use_new_robot_class:
      return self._clock() - self._last_reset_time
    else:
      return self._robot.GetTimeSinceReset()

  def get_time(self):
    """Gets the time reading from the clock source."""
    return self._clock()

  @property
  def observation(self):
    return self._observation_dict

  @property
  def pybullet_client(self):
    return self._pybullet_client

  @property
  def robot(self):
    return self._robot

  @property
  def num_action_repeat(self):
    return self._num_action_repeat

  @property
  def sim_time_step(self):
    return self._sim_time_step

  @property
  def env_step_counter(self):
    return self._env_step_counter

  @property
  def hard_reset(self):
    return self._hard_reset

  @property
  def last_action(self):
    return self._last_action

  @property
  def env_time_step(self):
    return self._env_time_step

  @property
  def data_dir(self):
    return self._data_dir

  @property
  def task(self):
    return self._task

  @property
  def robot_class(self):
    return self._robot_class

  @property
  def action_names(self):
    """Name of each action in the action space.

    By default this returns the actions the robot executes (e.g.
    "VELOCITY_elbow_joint"), but env wrappers may override this if they change
    the action space (e.g. if they convert twist to motor commands).

    Returns:
      Tuple of strings, the action names.
    """
    if self._use_new_robot_class:
      return self._robot.action_names
    return self._action_list

  @property
  def metric_logger(self):
    return self._metric_logger
