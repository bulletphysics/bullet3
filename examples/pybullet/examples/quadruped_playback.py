import pybullet as p
import time
import math
from datetime import datetime
from numpy import *
from pylab import *
import struct
import sys
import os, fnmatch
import argparse
from time import sleep
import pybullet_data


def readLogFile(filename, verbose=True):
  f = open(filename, 'rb')

  print('Opened'),
  print(filename)

  keys = f.readline().decode('utf8').rstrip('\n').split(',')
  fmt = f.readline().decode('utf8').rstrip('\n')

  # The byte number of one record
  sz = struct.calcsize(fmt)
  # The type number of one record
  ncols = len(fmt)

  if verbose:
    print('Keys:'),
    print(keys)
    print('Format:'),
    print(fmt)
    print('Size:'),
    print(sz)
    print('Columns:'),
    print(ncols)

  # Read data
  wholeFile = f.read()
  # split by alignment word
  chunks = wholeFile.split(b'\xaa\xbb')
  print("num chunks")
  print(len(chunks))

  log = list()
  for chunk in chunks:
    if len(chunk) == sz:
      values = struct.unpack(fmt, chunk)
      record = list()
      for i in range(ncols):
        record.append(values[i])
      log.append(record)

  return log


clid = p.connect(p.SHARED_MEMORY)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
log = readLogFile("LOG00076.TXT")

recordNum = len(log)
print('record num:'),
print(recordNum)
itemNum = len(log[0])
print('item num:'),
print(itemNum)

useRealTime = 0
fixedTimeStep = 0.001
speed = 10
amplitude = 0.8
jump_amp = 0.5
maxForce = 3.5
kp = .05
kd = .5

quadruped = 1
nJoints = p.getNumJoints(quadruped)
jointNameToId = {}
for i in range(nJoints):
  jointInfo = p.getJointInfo(quadruped, i)
  jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]

motor_front_rightR_joint = jointNameToId['motor_front_rightR_joint']
hip_front_rightR_link = jointNameToId['hip_front_rightR_link']
knee_front_rightR_link = jointNameToId['knee_front_rightR_link']
motor_front_rightL_joint = jointNameToId['motor_front_rightL_joint']
motor_front_rightL_link = jointNameToId['motor_front_rightL_link']
knee_front_rightL_link = jointNameToId['knee_front_rightL_link']
motor_front_leftR_joint = jointNameToId['motor_front_leftR_joint']
hip_front_leftR_link = jointNameToId['hip_front_leftR_link']
knee_front_leftR_link = jointNameToId['knee_front_leftR_link']
motor_front_leftL_joint = jointNameToId['motor_front_leftL_joint']
motor_front_leftL_link = jointNameToId['motor_front_leftL_link']
knee_front_leftL_link = jointNameToId['knee_front_leftL_link']
motor_back_rightR_joint = jointNameToId['motor_back_rightR_joint']
hip_rightR_link = jointNameToId['hip_rightR_link']
knee_back_rightR_link = jointNameToId['knee_back_rightR_link']
motor_back_rightL_joint = jointNameToId['motor_back_rightL_joint']
motor_back_rightL_link = jointNameToId['motor_back_rightL_link']
knee_back_rightL_link = jointNameToId['knee_back_rightL_link']
motor_back_leftR_joint = jointNameToId['motor_back_leftR_joint']
hip_leftR_link = jointNameToId['hip_leftR_link']
knee_back_leftR_link = jointNameToId['knee_back_leftR_link']
motor_back_leftL_joint = jointNameToId['motor_back_leftL_joint']
motor_back_leftL_link = jointNameToId['motor_back_leftL_link']
knee_back_leftL_link = jointNameToId['knee_back_leftL_link']

motorDir = [1, 1, 1, 1, 1, 1, 1, 1]
legnumbering = [
    motor_front_leftR_joint, motor_front_leftL_joint, motor_back_leftR_joint,
    motor_back_leftL_joint, motor_front_rightR_joint, motor_front_rightL_joint,
    motor_back_rightR_joint, motor_back_rightL_joint
]

for record in log:
  p.setJointMotorControl2(bodyIndex=quadruped,
                          jointIndex=legnumbering[0],
                          controlMode=p.POSITION_CONTROL,
                          targetPosition=motorDir[0] * record[7],
                          positionGain=kp,
                          velocityGain=kd,
                          force=maxForce)
  p.setJointMotorControl2(bodyIndex=quadruped,
                          jointIndex=legnumbering[1],
                          controlMode=p.POSITION_CONTROL,
                          targetPosition=motorDir[1] * record[8],
                          positionGain=kp,
                          velocityGain=kd,
                          force=maxForce)
  p.setJointMotorControl2(bodyIndex=quadruped,
                          jointIndex=legnumbering[2],
                          controlMode=p.POSITION_CONTROL,
                          targetPosition=motorDir[2] * record[9],
                          positionGain=kp,
                          velocityGain=kd,
                          force=maxForce)
  p.setJointMotorControl2(bodyIndex=quadruped,
                          jointIndex=legnumbering[3],
                          controlMode=p.POSITION_CONTROL,
                          targetPosition=motorDir[3] * record[10],
                          positionGain=kp,
                          velocityGain=kd,
                          force=maxForce)
  p.setJointMotorControl2(bodyIndex=quadruped,
                          jointIndex=legnumbering[4],
                          controlMode=p.POSITION_CONTROL,
                          targetPosition=motorDir[4] * record[11],
                          positionGain=kp,
                          velocityGain=kd,
                          force=maxForce)
  p.setJointMotorControl2(bodyIndex=quadruped,
                          jointIndex=legnumbering[5],
                          controlMode=p.POSITION_CONTROL,
                          targetPosition=motorDir[5] * record[12],
                          positionGain=kp,
                          velocityGain=kd,
                          force=maxForce)
  p.setJointMotorControl2(bodyIndex=quadruped,
                          jointIndex=legnumbering[6],
                          controlMode=p.POSITION_CONTROL,
                          targetPosition=motorDir[6] * record[13],
                          positionGain=kp,
                          velocityGain=kd,
                          force=maxForce)
  p.setJointMotorControl2(bodyIndex=quadruped,
                          jointIndex=legnumbering[7],
                          controlMode=p.POSITION_CONTROL,
                          targetPosition=motorDir[7] * record[14],
                          positionGain=kp,
                          velocityGain=kd,
                          force=maxForce)
  p.setGravity(0.000000, 0.000000, -10.000000)
  p.stepSimulation()
  p.stepSimulation()
  sleep(0.01)
