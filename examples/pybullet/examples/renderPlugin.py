
import pybullet as p
p.connect(p.GUI)
plugin = p.loadPlugin("e:/develop/bullet3/bin/pybullet_tinyRendererPlugin_vs2010_x64_debug.dll","_tinyRendererPlugin")
print("plugin=",plugin)
p.loadURDF("r2d2.urdf")

while (1):
	p.getCameraImage(320,200)