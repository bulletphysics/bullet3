r"""An example to run of the minitaur gym environment with sine gaits.
"""

import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import math
import numpy as np
from pybullet_envs.bullet import minitaur_gym_env
import argparse
from pybullet_envs.bullet import minitaur_env_randomizer


def ResetPoseExample():
  """An example that the minitaur stands still using the reset pose."""
  steps = 1000
  randomizer = (minitaur_env_randomizer.MinitaurEnvRandomizer())
  environment = minitaur_gym_env.MinitaurBulletEnv(render=True,
                                                   leg_model_enabled=False,
                                                   motor_velocity_limit=np.inf,
                                                   pd_control_enabled=True,
                                                   accurate_motor_model_enabled=True,
                                                   motor_overheat_protection=True,
                                                   env_randomizer=randomizer,
                                                   hard_reset=False)
  action = [math.pi / 2] * 8
  for _ in range(steps):
    _, _, done, _ = environment.step(action)
    if done:
      break
  environment.reset()


def MotorOverheatExample():
  """An example of minitaur motor overheat protection is triggered.

  The minitaur is leaning forward and the motors are getting obove threshold
  torques. The overheat protection will be triggered in ~1 sec.
  """

  environment = minitaur_gym_env.MinitaurBulletEnv(render=True,
                                                   leg_model_enabled=False,
                                                   motor_velocity_limit=np.inf,
                                                   motor_overheat_protection=True,
                                                   accurate_motor_model_enabled=True,
                                                   motor_kp=1.20,
                                                   motor_kd=0.00,
                                                   on_rack=False)

  action = [.0] * 8
  for i in range(8):
    action[i] = .0 - 0.1 * (-1 if i % 2 == 0 else 1) * (-1 if i < 4 else 1)

  steps = 500
  actions_and_observations = []
  for step_counter in range(steps):
    # Matches the internal timestep.
    time_step = 0.01
    t = step_counter * time_step
    current_row = [t]
    current_row.extend(action)

    observation, _, _, _ = environment.step(action)
    current_row.extend(observation.tolist())
    actions_and_observations.append(current_row)
  environment.reset()


def SineStandExample():
  """An example of minitaur standing and squatting on the floor.

  To validate the accurate motor model we command the robot and sit and stand up
  periodically in both simulation and experiment. We compare the measured motor
  trajectories, torques and gains.
  """
  environment = minitaur_gym_env.MinitaurBulletEnv(render=True,
                                                   leg_model_enabled=False,
                                                   motor_velocity_limit=np.inf,
                                                   motor_overheat_protection=True,
                                                   accurate_motor_model_enabled=True,
                                                   motor_kp=1.20,
                                                   motor_kd=0.02,
                                                   on_rack=False)
  steps = 1000
  amplitude = 0.5
  speed = 3

  actions_and_observations = []

  for step_counter in range(steps):
    # Matches the internal timestep.
    time_step = 0.01
    t = step_counter * time_step
    current_row = [t]

    action = [math.sin(speed * t) * amplitude + math.pi / 2] * 8
    current_row.extend(action)

    observation, _, _, _ = environment.step(action)
    current_row.extend(observation.tolist())
    actions_and_observations.append(current_row)

  environment.reset()


def SinePolicyExample():
  """An example of minitaur walking with a sine gait."""
  randomizer = (minitaur_env_randomizer.MinitaurEnvRandomizer())
  environment = minitaur_gym_env.MinitaurBulletEnv(render=True,
                                                   motor_velocity_limit=np.inf,
                                                   pd_control_enabled=True,
                                                   hard_reset=False,
                                                   env_randomizer=randomizer,
                                                   on_rack=False)
  sum_reward = 0
  steps = 20000
  amplitude_1_bound = 0.1
  amplitude_2_bound = 0.1
  speed = 1

  for step_counter in range(steps):
    time_step = 0.01
    t = step_counter * time_step

    amplitude1 = amplitude_1_bound
    amplitude2 = amplitude_2_bound
    steering_amplitude = 0
    if t < 10:
      steering_amplitude = 0.1
    elif t < 20:
      steering_amplitude = -0.1
    else:
      steering_amplitude = 0

    # Applying asymmetrical sine gaits to different legs can steer the minitaur.
    a1 = math.sin(t * speed) * (amplitude1 + steering_amplitude)
    a2 = math.sin(t * speed + math.pi) * (amplitude1 - steering_amplitude)
    a3 = math.sin(t * speed) * amplitude2
    a4 = math.sin(t * speed + math.pi) * amplitude2
    action = [a1, a2, a2, a1, a3, a4, a4, a3]
    _, reward, done, _ = environment.step(action)
    sum_reward += reward
    if done:
      break
  environment.reset()


def main():
  parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('--env',
                      help='environment ID (0==sine, 1==stand, 2=reset, 3=overheat)',
                      type=int,
                      default=0)
  args = parser.parse_args()
  print("--env=" + str(args.env))

  if (args.env == 0):
    SinePolicyExample()
  if (args.env == 1):
    SineStandExample()
  if (args.env == 2):
    ResetPoseExample()
  if (args.env == 3):
    MotorOverheatExample()


if __name__ == '__main__':
  main()
