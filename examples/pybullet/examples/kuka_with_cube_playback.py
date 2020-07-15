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
  log = list()
  for chunk in chunks:
    if len(chunk) == sz:
      values = struct.unpack(fmt, chunk)
      record = list()
      for i in range(ncols):
        record.append(values[i])
      log.append(record)

  return log


#clid = p.connect(p.SHARED_MEMORY)
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", [0, 0, -0.3])
p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 1])
p.loadURDF("cube.urdf", [2, 2, 5])
p.loadURDF("cube.urdf", [-2, -2, 5])
p.loadURDF("cube.urdf", [2, -2, 5])

log = readLogFile("LOG0001.txt")

recordNum = len(log)
itemNum = len(log[0])
print('record num:'),
print(recordNum)
print('item num:'),
print(itemNum)

for record in log:
  Id = record[2]
  pos = [record[3], record[4], record[5]]
  orn = [record[6], record[7], record[8], record[9]]
  p.resetBasePositionAndOrientation(Id, pos, orn)
  numJoints = p.getNumJoints(Id)
  for i in range(numJoints):
    jointInfo = p.getJointInfo(Id, i)
    qIndex = jointInfo[3]
    if qIndex > -1:
      p.resetJointState(Id, i, record[qIndex - 7 + 17])
  sleep(0.0005)
