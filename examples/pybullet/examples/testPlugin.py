import pybullet as p
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plugin = p.loadPlugin("D:/develop/bullet3/bin/pybullet_testplugin_vs2010_x64_debug.dll",
                      "_testPlugin")
print("plugin=", plugin)
p.executePluginCommand(plugin, "r2d2.urdf", [1, 2, 3], [50.0, 3.3])

while (1):
  p.getCameraImage(320, 200)
  p.stepSimulation()
