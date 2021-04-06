import pybullet as p
import time
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
fileIO = p.loadPlugin("fileIOPlugin")
if (fileIO >= 0):
  #we can have a zipfile (pickup.zip) inside a zipfile (pickup2.zip)
  p.executePluginCommand(fileIO, pybullet_data.getDataPath()+"/pickup2.zip", [p.AddFileIOAction, p.ZipFileIO])
  p.executePluginCommand(fileIO, "pickup.zip", [p.AddFileIOAction, p.ZipFileIO])
  objs = p.loadSDF("pickup/model.sdf")
  dobot = objs[0]
  p.changeVisualShape(dobot, -1, rgbaColor=[1, 1, 1, 1])

else:
  print("fileIOPlugin is disabled.")

p.setPhysicsEngineParameter(enableFileCaching=False)

while (1):
  p.stepSimulation()
  time.sleep(1. / 240.)
