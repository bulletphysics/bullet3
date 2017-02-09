from minitaur import Minitaur
import time
import numpy as np
import pybullet as p
import math
import sys

minitaur = None

def current_position():
  global minitaur
  position = minitaur.getBasePosition()
  return np.asarray(position)

def is_fallen():
  global minitaur
  orientation = minitaur.getBaseOrientation()
  rotMat = p.getMatrixFromQuaterion(orientation)
  localUp = rotMat[6:]
  return np.dot(np.asarray([0, 0, 1]), np.asarray(localUp)) < 0


def evaluate_params_hop(params, urdfRoot='', timeStep=0.01, maxNumSteps=1000, sleepTime=0):
  print('start evaluation')
  beforeTime = time.time()
  p.resetSimulation()

  p.setTimeStep(timeStep)
  p.loadURDF("%s/plane.urdf" % urdfRoot)
  p.setGravity(0,0,-10)

  amplitude = params[0]
  speed = params[1]

  global minitaur
  minitaur = Minitaur(urdfRoot)
  start_position = current_position()
  last_position = None  # for tracing line

  for i in range(maxNumSteps):
    a1 = math.sin(i*speed)*amplitude+1.57
    a2 = math.sin(i*speed+3.14)*amplitude+1.57
    joint_values = [a1, 1.57, a2, 1.57, 1.57, a1, 1.57, a2]
    minitaur.applyAction(joint_values)
    p.stepSimulation()
    if (is_fallen()):
      break

    if i % 100 == 0:
      sys.stdout.write('.')
      sys.stdout.flush()
    time.sleep(sleepTime)

  print(' ')

  final_distance = np.linalg.norm(start_position - current_position())
  elapsedTime = time.time() - beforeTime
  print ("trial for amplitude", amplitude, "speed", speed, "final_distance", final_distance, "elapsed_time", elapsedTime)
  return final_distance
