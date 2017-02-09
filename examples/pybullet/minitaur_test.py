import pybullet as p

from minitaur import Minitaur
import minitaur_evaluate
import time
import math
import numpy as np

def main(unused_args):
  timeStep = 0.01
  c = p.connect(p.SHARED_MEMORY)
  if (c<0):
      c = p.connect(p.GUI)

  amplitude = 0.24795664427
  speed = 0.2860877729434

  final_distance = minitaur_evaluate.evaluate_params_hop(params=[amplitude, speed], timeStep=timeStep, sleepTime=timeStep)
  print(final_distance)

  # p.resetSimulation()
  # p.setTimeStep(timeStep)
  # p.loadURDF("plane.urdf")
  # p.setGravity(0,0,-10)

  # minitaur = Minitaur()

  # for i in range(1000):
  #   a1 = math.sin(i*speed)*amplitude+1.57
  #   a2 = math.sin(i*speed+3.14)*amplitude+1.57
  #   joint_values = [a1, 1.57, a2, 1.57, 1.57, a1, 1.57, a2]
  #   minitaur.applyAction(joint_values)
  #   torques = minitaur.getMotorTorques()
  #   print(torques)
  #   p.stepSimulation()
  #   time.sleep(timeStep)
  # final_distance = np.linalg.norm(np.asarray(minitaur.getBasePosition()))
  # print(final_distance)

main(0)
