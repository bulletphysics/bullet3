import json

class MotionCaptureData(object):
  def __init__(self):
    self.Reset()

  def Reset(self):
    self._motion_data = []

  def Load(self, path):
    with open(path, 'r') as f:
      self._motion_data = json.load(f)

  def NumFrames(self):
    return  len(self._motion_data['Frames'])

  def KeyFrameDuraction(self):
  	return self._motion_data['Frames'][0][0]
 
