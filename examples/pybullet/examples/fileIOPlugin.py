import pybullet as p
import time

p.connect(p.GUI)
fileIO = p.loadPlugin("fileIOPlugin")
if (fileIO>=0):
	p.executePluginCommand(fileIO, "pickup.zip", [p.AddFileIOAction, p.ZipFileIO])
	objs= p.loadSDF("pickup/model.sdf")
	dobot =objs[0]
	p.changeVisualShape(dobot,-1,rgbaColor=[1,1,1,1])
		
else:
	print("fileIOPlugin is disabled.")


p.setPhysicsEngineParameter(enableFileCaching=False)

while (1):
	p.stepSimulation()
	time.sleep(1./240.)