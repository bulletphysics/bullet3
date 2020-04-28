import pybullet as p
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plugin = p.loadPlugin("d:/develop/bullet3/bin/pybullet_testplugin_vs2010_x64_debug.dll",
                      "_testPlugin")
print("plugin=", plugin)
p.loadURDF("r2d2.urdf")

while (1):
  p.getCameraImage(320, 200)
