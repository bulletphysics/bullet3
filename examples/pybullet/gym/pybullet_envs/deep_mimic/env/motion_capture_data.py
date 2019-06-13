import json
import math


class MotionCaptureData(object):

  def __init__(self):
    self.Reset()

  def Reset(self):
    self._motion_data = []

  def Load(self, path):
    with open(path, 'r') as f:
      self._motion_data = json.load(f)

  def NumFrames(self):
    return len(self._motion_data['Frames'])

  def KeyFrameDuraction(self):
    return self._motion_data['Frames'][0][0]

  def getCycleTime(self):
    keyFrameDuration = self.KeyFrameDuraction()
    cycleTime = keyFrameDuration * (self.NumFrames() - 1)
    return cycleTime

  def calcCycleCount(self, simTime, cycleTime):
    phases = simTime / cycleTime
    count = math.floor(phases)
    loop = True
    #count = (loop) ? count : cMathUtil::Clamp(count, 0, 1);
    return count

  def computeCycleOffset(self):
    firstFrame = 0
    lastFrame = self.NumFrames() - 1
    frameData = self._motion_data['Frames'][0]
    frameDataNext = self._motion_data['Frames'][lastFrame]

    basePosStart = [frameData[1], frameData[2], frameData[3]]
    basePosEnd = [frameDataNext[1], frameDataNext[2], frameDataNext[3]]
    self._cycleOffset = [
        basePosEnd[0] - basePosStart[0], basePosEnd[1] - basePosStart[1],
        basePosEnd[2] - basePosStart[2]
    ]
    return self._cycleOffset
