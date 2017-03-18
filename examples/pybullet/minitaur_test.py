import sys
#some python interpreters need '.' added
sys.path.append(".")

import pybullet as p
from minitaur import Minitaur
from minitaur_evaluate import *

import time
import math
import numpy as np

def main(unused_args):
  timeStep = 0.01
  c = p.connect(p.SHARED_MEMORY)
  if (c<0):
      c = p.connect(p.GUI)

  params = [0.1903581461951056, 0.0006732219568880068, 0.05018085615283363, 3.219916795483583, 6.2406418167980595, 4.189869754607539]
  evaluate_func = 'evaluate_desired_motorAngle_2Amplitude4Phase'
  energy_weight = 0.01

  finalReturn = evaluate_params(evaluateFunc = evaluate_func, params=params, objectiveParams=[energy_weight], timeStep=timeStep, sleepTime=timeStep)

  print(finalReturn)

main(0)
