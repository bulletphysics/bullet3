from minitaur import Minitaur
import pybullet as p
import numpy as np
import time
import sys
import math

minitaur = None

evaluate_func_map = dict()


def current_position():
  global minitaur
  position = minitaur.getBasePosition()
  return np.asarray(position)


def is_fallen():
  global minitaur
  orientation = minitaur.getBaseOrientation()
  rotMat = p.getMatrixFromQuaternion(orientation)
  localUp = rotMat[6:]
  return np.dot(np.asarray([0, 0, 1]), np.asarray(localUp)) < 0


def evaluate_desired_motorAngle_8Amplitude8Phase(i, params):
  nMotors = 8
  speed = 0.35
  for jthMotor in range(nMotors):
    joint_values[jthMotor] = math.sin(i * speed +
                                      params[nMotors + jthMotor]) * params[jthMotor] * +1.57
  return joint_values


def evaluate_desired_motorAngle_2Amplitude4Phase(i, params):
  speed = 0.35
  phaseDiff = params[2]
  a0 = math.sin(i * speed) * params[0] + 1.57
  a1 = math.sin(i * speed + phaseDiff) * params[1] + 1.57
  a2 = math.sin(i * speed + params[3]) * params[0] + 1.57
  a3 = math.sin(i * speed + params[3] + phaseDiff) * params[1] + 1.57
  a4 = math.sin(i * speed + params[4] + phaseDiff) * params[1] + 1.57
  a5 = math.sin(i * speed + params[4]) * params[0] + 1.57
  a6 = math.sin(i * speed + params[5] + phaseDiff) * params[1] + 1.57
  a7 = math.sin(i * speed + params[5]) * params[0] + 1.57
  joint_values = [a0, a1, a2, a3, a4, a5, a6, a7]
  return joint_values


def evaluate_desired_motorAngle_hop(i, params):
  amplitude = params[0]
  speed = params[1]
  a1 = math.sin(i * speed) * amplitude + 1.57
  a2 = math.sin(i * speed + 3.14) * amplitude + 1.57
  joint_values = [a1, 1.57, a2, 1.57, 1.57, a1, 1.57, a2]
  return joint_values


evaluate_func_map[
    'evaluate_desired_motorAngle_8Amplitude8Phase'] = evaluate_desired_motorAngle_8Amplitude8Phase
evaluate_func_map[
    'evaluate_desired_motorAngle_2Amplitude4Phase'] = evaluate_desired_motorAngle_2Amplitude4Phase
evaluate_func_map['evaluate_desired_motorAngle_hop'] = evaluate_desired_motorAngle_hop


def evaluate_params(evaluateFunc,
                    params,
                    objectiveParams,
                    urdfRoot='',
                    timeStep=0.01,
                    maxNumSteps=10000,
                    sleepTime=0):
  print('start evaluation')
  beforeTime = time.time()
  p.resetSimulation()

  p.setTimeStep(timeStep)
  p.loadURDF("%s/plane.urdf" % urdfRoot)
  p.setGravity(0, 0, -10)

  global minitaur
  minitaur = Minitaur(urdfRoot)
  start_position = current_position()
  last_position = None  # for tracing line
  total_energy = 0

  for i in range(maxNumSteps):
    torques = minitaur.getMotorTorques()
    velocities = minitaur.getMotorVelocities()
    total_energy += np.dot(np.fabs(torques), np.fabs(velocities)) * timeStep

    joint_values = evaluate_func_map[evaluateFunc](i, params)
    minitaur.applyAction(joint_values)
    p.stepSimulation()
    if (is_fallen()):
      break

    if i % 100 == 0:
      sys.stdout.write('.')
      sys.stdout.flush()
    time.sleep(sleepTime)

  print(' ')

  alpha = objectiveParams[0]
  final_distance = np.linalg.norm(start_position - current_position())
  finalReturn = final_distance - alpha * total_energy
  elapsedTime = time.time() - beforeTime
  print("trial for ", params, " final_distance", final_distance, "total_energy", total_energy,
        "finalReturn", finalReturn, "elapsed_time", elapsedTime)
  return finalReturn
