"""Common tools and functionalities used in different tasks."""
import numpy as np


def calculate_target_speed_at_timestep(speed_stages):
  """Interpolates the speed based on a speed profile and simulation steps.

  Args:
    speed_stages: The list of timesteps (in increasing order) and speed (float
      or a list of floats for x and y components) at that specific timestep.
      Example formats can be found on task_utils_test.py and at the header of
      the speed_reward_task.py.

  Returns:
    Target speed for the specific step (meters per simulation step).
  Raises:
    ValueError if the input is not in the expected format.
  """
  if len(speed_stages) != 2 or len(speed_stages[0]) != len(speed_stages[1]):
    raise ValueError('Speed stages for the task is not in correct format!')
  steps = np.array(speed_stages[0])
  speeds = np.array(speed_stages[1])
  num_steps = steps[-1]
  if len(speeds.shape) == 1:
    return np.interp(range(num_steps), steps, speeds)
  speed_at_timestep = np.interp(range(num_steps), steps, speeds[:, 0]).reshape(
      (num_steps, 1))
  speed_at_timestep = speed_at_timestep.reshape((num_steps, 1))
  if speeds.shape[1] == 2:
    speed_y = np.interp(range(num_steps), steps, speeds[:, 1]).reshape(
        (num_steps, 1))
    speed_at_timestep = np.concatenate((speed_at_timestep, speed_y), axis=1)
  else:
    speed_at_timestep = np.concatenate(
        (speed_at_timestep, np.zeros((num_steps, 1))), axis=1)
  return speed_at_timestep


def calculate_distance(vector_1, vector_2):
  """Calculates the distance between 2 vectors.

  This is used to calculate distance between 2 points in 3D space as well as
  distances between two orientation vectors.

  Args:
    vector_1: First vector that will be used for comparison with the other.
    vector_2: Second vector used for comparison.

  Returns:
    Distance between the two vectors represented by a float.
  """
  return np.linalg.norm(np.array(vector_1) - np.array(vector_2))


def turn_angle(new_vector, reference_vector):
  """Calculates the change in orientation of the two vectors.

    This is used to calculate the relative angle perception for the
    robot.

  Args:
    new_vector: The front vector of the robot at current timestep.
    reference_vector: The front vector of the robot at previous timestep.

  Returns:
    Angle representing the change in orientation of the robot projected to
    x-y plane.
  """
  # Project the vectors to x-y plane
  v1 = np.resize(new_vector, 3)
  v2 = np.resize(reference_vector, 3)
  v1[2] = 0
  v2[2] = 0
  # Calculate the right hand rotation between two vectors using z vector
  # (0,0,-1) as reference cross product vector.
  # Compatible with the yaw rotation of pyBullet.
  return np.arctan2(np.dot(np.cross(v2, v1), (0, 0, -1)), np.dot(v1, v2))


def front_vector(pybullet_client, orientation):
  """Calculates the front vector of the robot on x-y plane.

  Args:
    pybullet_client: Pybullet client instantiation.
    orientation: Orientation of the robot in quaternion form.

  Returns:
    3D vector where z component is set to 0.
  """
  rot_matrix = pybullet_client.getMatrixFromQuaternion(orientation)
  return [rot_matrix[0], -rot_matrix[1], 0]


def calculate_estimated_energy_consumption(motor_torques, motor_velocities,
                                           sim_time_step, num_action_repeat):
  """Calculates energy consumption based on the args listed.

  Args:
    motor_torques: Torques of all the motors
    motor_velocities: Velocities of all the motors.
    sim_time_step: Simulation time step length (seconds).
    num_action_repeat: How many steps the simulation repeats the same action.

  Returns:
    Total energy consumption of all the motors (watts).
  """
  return np.abs(np.dot(motor_torques,
                       motor_velocities)) * sim_time_step * num_action_repeat
