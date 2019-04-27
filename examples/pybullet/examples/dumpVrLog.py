import time
import math
from datetime import datetime
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
  if verbose:
    print("num chunks:")
    print(len(chunks))
  chunkIndex = 0
  for chunk in chunks:
    print("len(chunk)=", len(chunk), " sz = ", sz)
    if len(chunk) == sz:
      print("chunk #", chunkIndex)
      chunkIndex = chunkIndex + 1
      values = struct.unpack(fmt, chunk)
      record = list()
      for i in range(ncols):
        record.append(values[i])
        if verbose:
          print("    ", keys[i], "=", values[i])

      log.append(record)

  return log


numArgs = len(sys.argv)

print('Number of arguments:', numArgs, 'arguments.')
print('Argument List:', str(sys.argv))
fileName = "data/example_log_vr.bin"

if (numArgs > 1):
  fileName = sys.argv[1]

print("filename=")
print(fileName)

verbose = True

log = readLogFile(fileName, verbose)

# the index of the first integer in the vr log file for packed buttons
firstPackedButtonIndex = 13
# the number of packed buttons in one integer
numGroupedButtons = 10
# the number of integers for packed buttons
numPackedButtons = 7
# the mask to get the button state
buttonMask = 7

for record in log:
  # indices of buttons that are down
  buttonDownIndices = []
  # indices of buttons that are triggered
  buttonTriggeredIndices = []
  # indices of buttons that are released
  buttonReleasedIndices = []
  buttonIndex = 0
  for packedButtonIndex in range(firstPackedButtonIndex,
                                 firstPackedButtonIndex + numPackedButtons):
    for packButtonShift in range(numGroupedButtons):
      buttonEvent = buttonMask & record[packedButtonIndex]
      if buttonEvent & 1:
        buttonDownIndices.append(buttonIndex)
      elif buttonEvent & 2:
        buttonTriggeredIndices.append(buttonIndex)
      elif buttonEvent & 4:
        buttonReleasedIndices.append(buttonIndex)
      record[packedButtonIndex] = record[packedButtonIndex] >> 3
      buttonIndex += 1
  if len(buttonDownIndices) or len(buttonTriggeredIndices) or len(buttonReleasedIndices):
    print('timestamp: ', record[1])
    print('button is down: ', buttonDownIndices)
    print('button is triggered: ', buttonTriggeredIndices)
    print('button is released: ', buttonReleasedIndices)
