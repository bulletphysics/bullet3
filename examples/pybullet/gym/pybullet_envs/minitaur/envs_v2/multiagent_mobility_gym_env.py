"""This file implements the locomotion gym env."""
# pylint: disable=dangerous-default-value

import atexit
import collections
import time

import gin
from gym import spaces
import numpy as np

from pybullet_utils import bullet_client
from pybullet_envs.minitaur.envs import minitaur_logging
from pybullet_envs.minitaur.envs import minitaur_logging_pb2
from pybullet_envs.minitaur.envs_v2 import locomotion_gym_env
from pybullet_envs.minitaur.envs_v2.scenes import scene_base
from pybullet_envs.minitaur.envs_v2.sensors import sensor
from pybullet_envs.minitaur.envs_v2.sensors import space_utils
import pybullet

_ACTION_EPS = 0.01
_NUM_SIMULATION_ITERATION_STEPS = 300
_LOG_BUFFER_LENGTH = 5000


@gin.configurable
class MultiagentMobilityGymEnv(locomotion_gym_env.LocomotionGymEnv):
  """The gym environment for the locomotion tasks."""
  metadata = {
      'render.modes': ['human', 'rgb_array'],
      'video.frames_per_second': 100
  }

  def __init__(self,
               gym_config,
               robot_classes,
               scene: scene_base.SceneBase = scene_base.SceneBase(),
               sensors=None,
               tasks=[],
               global_task=None,
               single_reward=False,
               env_randomizers=None):
    """Initializes the locomotion gym environment.

    Args:
      gym_config: An instance of LocomotionGymConfig.
      robot_classes: A list of robot classes. We provide a class rather than an
        instance due to hard_reset functionality. Parameters are expected to be
        configured with gin.
      scene: An object for managing the robot's surroundings.
      sensors: A list of environmental sensors for observation. This does not
        include on-robot sensors.
      tasks: A list of callable function/class to calculate the reward and
        termination condition. Takes the gym env as the argument when calling.
      global_task: A callable function/class to calculate the reward and
        termination condition for all robots. Takes the gym env as the argument
        when calling.
      single_reward: Whether the environment returns a single reward for all
        agents or a dictionary.
      env_randomizers: A list of EnvRandomizer(s). An EnvRandomizer may
        randomize the physical property of minitaur, change the terrrain during
        reset(), or add perturbation forces during step().

    Raises:
      ValueError: If the num_action_repeat is less than 1, or if number of
        unique robot names do not match the number of robot classes.

    """
    # TODO(sehoonha) split observation and full-state sensors (b/129858214)

    # Makes sure that close() is always called to flush out the logs to the
    # disk.
    atexit.register(self.close)
    self.seed()
    self._gym_config = gym_config
    self._robot_classes = robot_classes
    # Checking uniqueness of names and number of names
    self._scene = scene
    # TODO(sehoonha) change the data structure to dictionary
    # TODO(b/144521291) make sure sensors have their own robot names
    self._sensors = sensors if sensors is not None else list()
    self._log_path = gym_config.log_path
    self._logging = minitaur_logging.MinitaurLogging(self._log_path)
    self._episode_proto = minitaur_logging_pb2.MinitaurEpisode()
    self._data_dir = gym_config.data_dir

    # A dictionary containing the objects in the world other than the robot.
    self._tasks = tasks
    self._global_task = global_task
    self._single_reward = single_reward

    self._env_randomizers = env_randomizers if env_randomizers else []

    # This is a workaround due to the issue in b/130128505#comment5
    for task in self._tasks:
      if isinstance(task, sensor.Sensor):
        self._sensors.append(task)
    if global_task and isinstance(global_task, sensor.Sensor):
      self._sensors.append(global_task)

    # Simulation related parameters.
    self._num_action_repeat = gym_config.simulation_parameters.num_action_repeat
    self._on_rack = gym_config.simulation_parameters.robot_on_rack
    if self._num_action_repeat < 1:
      raise ValueError('number of action repeats should be at least 1.')
    self._sim_time_step = gym_config.simulation_parameters.sim_time_step_s
    self._env_time_step = self._num_action_repeat * self._sim_time_step
    self._env_step_counter = 0

    # TODO(b/73829334): Fix the value of self._num_bullet_solver_iterations.
    self._num_bullet_solver_iterations = int(_NUM_SIMULATION_ITERATION_STEPS /
                                             self._num_action_repeat)
    self._is_render = gym_config.simulation_parameters.enable_rendering

    # The wall-clock time at which the last frame is rendered.
    self._last_frame_time = 0.0
    if self._is_render:
      self._pybullet_client = bullet_client.BulletClient(
          connection_mode=pybullet.GUI)
    else:
      self._pybullet_client = bullet_client.BulletClient()

    if gym_config.simulation_parameters.egl_rendering:
      self._pybullet_client.loadPlugin('eglRendererPlugin')
    self._pybullet_client.enable_cns()

    # If enabled, save the performance profile to profiling_path
    # Use Google Chrome about://tracing to open the file
    if gym_config.profiling_path is not None:
      self._profiling_slot = self._pybullet_client.startStateLogging(
          self._pybullet_client.STATE_LOGGING_PROFILE_TIMINGS,
          gym_config.profiling_path)
      self._profiling_counter = 10
    else:
      self._profiling_slot = -1
    # Build the action space. The action space must be compatible with the
    # robot configuration.

    # The action list contains the name of all actions.
    # TODO(b/144479707): Allow robots to set the action space automatically.

    action_space = collections.OrderedDict()
    for robot_name, action in gym_config.actions.items():
      action_lower_bound = []
      action_upper_bound = []
      for action_scalar in action:
        action_upper_bound.append(action_scalar.upper_bound)
        action_lower_bound.append(action_scalar.lower_bound)
      action_space[robot_name] = spaces.Box(
          np.asarray(action_lower_bound),
          np.asarray(action_upper_bound),
          dtype=np.float32)
    self.action_space = spaces.Dict(action_space)

    # Set the default render options.
    self._camera_dist = gym_config.simulation_parameters.camera_distance
    self._camera_yaw = gym_config.simulation_parameters.camera_yaw
    self._camera_pitch = gym_config.simulation_parameters.camera_pitch
    self._render_width = gym_config.simulation_parameters.render_width
    self._render_height = gym_config.simulation_parameters.render_height

    self._hard_reset = True
    self.reset()

    self._hard_reset = gym_config.simulation_parameters.enable_hard_reset

    # Construct the observation space from the list of sensors. Note that we
    # will reconstruct the observation_space after the robot is created.
    self.observation_space = (
        space_utils.convert_sensors_to_gym_space_dictionary(self.all_sensors()))

  def close(self):
    if self._log_path is not None:
      self._logging.save_episode(self._episode_proto)

    for robot in self._robots:
      robot.Terminate()

  def all_sensors(self):
    """Returns all robot and environmental sensors."""
    robot_sensors = []
    for robot in self._robots:
      robot_sensors += robot.GetAllSensors()
    return robot_sensors + self._sensors

  @gin.configurable('multiagent_mobility_gym_env.MultiagentMobilityGymEnv.reset'
                   )
  def reset(self,
            initial_motor_angles=None,
            reset_duration=1.0,
            reset_visualization_camera=True):
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
    if self._is_render:
      self._pybullet_client.configureDebugVisualizer(
          self._pybullet_client.COV_ENABLE_RENDERING, 0)

    # Clear the simulation world and rebuild the robot interface.
    if self._hard_reset:
      self._pybullet_client.resetSimulation()
      self._pybullet_client.setPhysicsEngineParameter(
          numSolverIterations=self._num_bullet_solver_iterations)
      self._pybullet_client.setTimeStep(self._sim_time_step)
      self._pybullet_client.setGravity(0, 0, -10)

      # Rebuild the world.
      self._scene.build_scene(self._pybullet_client)

      # Rebuild the robots
      # TODO(b/144545080): Make this scale to more than two agents
      # Have multiple robot classes as a list.
      self._robots = []
      for robot_class in self._robot_classes:

        self._robots.append(
            robot_class(
                pybullet_client=self._pybullet_client,
                # TODO(rosewang): Remove on rack in multiagent acase
                on_rack=self._on_rack))

    # Reset the pose of the robot.
    for robot in self._robots:
      robot.Reset(
          reload_urdf=False,
          default_motor_angles=initial_motor_angles,
          reset_time=reset_duration)

    self._env_step_counter = 0
    self._pybullet_client.resetDebugVisualizerCamera(self._camera_dist,
                                                     self._camera_yaw,
                                                     self._camera_pitch,
                                                     [0, 0, 0])

    # Flush the logs to disc and reinitialize the logging system.
    if self._log_path is not None:
      self._logging.save_episode(self._episode_proto)
      self._episode_proto = minitaur_logging_pb2.MinitaurEpisode()
      minitaur_logging.preallocate_episode_proto(self._episode_proto,
                                                 _LOG_BUFFER_LENGTH,
                                                 self._robots[0])
    self._pybullet_client.setPhysicsEngineParameter(enableConeFriction=0)
    self._env_step_counter = 0
    if reset_visualization_camera:
      self._pybullet_client.resetDebugVisualizerCamera(self._camera_dist,
                                                       self._camera_yaw,
                                                       self._camera_pitch,
                                                       [0, 0, 0])

    self._last_action = {
        robot_name: np.zeros(space.shape)
        for robot_name, space in self.action_space.spaces.items()
    }

    if self._is_render:
      self._pybullet_client.configureDebugVisualizer(
          self._pybullet_client.COV_ENABLE_RENDERING, 1)

    for s in self.all_sensors():
      # set name
      if any([r.name in s.get_name() for r in self.robots]):
        robot = [r for r in self.robots if r.name in s.get_name()][0]
        s.set_robot(robot)

    for task in self._tasks:
      if hasattr(task, 'reset'):
        task.reset(self)
    if self._global_task and hasattr(self._global_task, 'reset'):
      self._global_task.reset(self)

    # Loop over all env randomizers.
    for env_randomizer in self._env_randomizers:
      env_randomizer.randomize_env(self)

    for s in self.all_sensors():
      s.on_reset(self)

    return self._get_observation()

  def get_robot(self, name):
    for robot in self.robots:
      if robot.name == name:
        return robot

  def _reward(self):
    """Returns a list of rewards.

    Returns:
      A list of rewards corresponding to each robot and their task.
    """
    global_reward = 0
    if self._global_task:
      global_reward = self._global_task(self)
    if self._single_reward:  # Needed for tfagents compatibility.
      if self._tasks:
        return min([task(self) + global_reward for task in self._tasks])
      return 0
    else:
      if self._tasks:
        return [task(self) + global_reward for task in self._tasks]
      return [0 for _ in self.robots]

  def step(self, actions):
    """Step forward the simulation, given the actions.

    Args:
      actions: A dictionary of actions for all robots. The action for each robot
        can be joint angles for legged platforms like Laikago, and base
        velocity/steering for kinematic robots such like Fetch.

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
    self._last_base_position = [
        robot.GetBasePosition() for robot in self._robots
    ]
    self._last_action = actions

    if self._is_render:
      # Sleep, otherwise the computation takes less time than real time,
      # which will make the visualization like a fast-forward video.
      time_spent = time.time() - self._last_frame_time
      self._last_frame_time = time.time()
      time_to_sleep = self._env_time_step - time_spent
      if time_to_sleep > 0:
        time.sleep(time_to_sleep)
      camera_target = np.mean(
          [robot.GetBasePosition() for robot in self._robots], axis=0)

      # Also keep the previous orientation of the camera set by the user.
      [yaw, pitch,
       dist] = self._pybullet_client.getDebugVisualizerCamera()[8:11]
      self._pybullet_client.resetDebugVisualizerCamera(dist, yaw, pitch,
                                                       camera_target)

    for env_randomizer in self._env_randomizers:
      env_randomizer.randomize_step(self)

    # Stepping broken down into their parts
    for robot in self._robots:
      robot.PreStepPerStep(actions)

    for _ in range(self._num_action_repeat):
      for robot in self._robots:
        robot.PreStepPerActionRepeat(actions)

      self._pybullet_client.stepSimulation()

      for robot in self._robots:
        robot.PostStepPerActionRepeat()

    for robot in self._robots:
      robot.PostStepPerStep()

    if self._profiling_slot >= 0:
      self._profiling_counter -= 1
      if self._profiling_counter == 0:
        self._pybullet_client.stopStateLogging(self._profiling_slot)
    if self._log_path is not None:
      minitaur_logging.update_episode_proto(self._episode_proto,
                                            self._robots[0], actions,
                                            self._env_step_counter)
    reward = self._reward()

    for s in self.all_sensors():
      s.on_step(self)

    for task in self._tasks:
      if hasattr(task, 'update'):
        task.update(self)
    if self._global_task and hasattr(self._global_task, 'update'):
      self._global_task.update(self)

    done = self._termination()
    self._env_step_counter += 1
    if done:
      for robot in self._robots:
        robot.Terminate()
    return self._get_observation(), reward, done, {}

  def render(self, mode='rgb_array'):
    if mode != 'rgb_array':
      raise ValueError('Unsupported render mode:{}'.format(mode))
    base_pos = np.mean([robot.GetBasePosition() for robot in self._robots],
                       axis=0)
    view_matrix = self._pybullet_client.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=base_pos,
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
    (_, _, px, _, _) = self._pybullet_client.getCameraImage(
        width=self._render_width,
        height=self._render_height,
        renderer=self._pybullet_client.ER_BULLET_HARDWARE_OPENGL,
        viewMatrix=view_matrix,
        projectionMatrix=proj_matrix)
    rgb_array = np.array(px)
    rgb_array = rgb_array[:, :, :3]
    return rgb_array

  def _termination(self):
    if not all([robot.is_safe for robot in self._robots]):
      return True

    if self._tasks:
      return (self._global_task and self._global_task.done(self)) or any(
          [task.done(self) for task in self._tasks])

    for s in self.all_sensors():
      s.on_terminate(self)

    return False

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
    self._num_bullet_solver_iterations = (
        _NUM_SIMULATION_ITERATION_STEPS / self._num_action_repeat)
    self._pybullet_client.setPhysicsEngineParameter(
        numSolverIterations=self._num_bullet_solver_iterations)
    self._pybullet_client.setTimeStep(self._sim_time_step)
    for robot in self._robots:
      robot.SetTimeSteps(self._num_action_repeat, self._sim_time_step)

  def get_time_since_reset(self):
    """Get the time passed (in seconds) since the last reset.

    Returns:
      List of time in seconds since the last reset for each robot.
    """
    return self._robots[0].GetTimeSinceReset()

  @property
  def tasks(self):
    return self._tasks

  @property
  def robots(self):
    return self._robots

  @property
  def num_robots(self):
    return len(self._robots)
