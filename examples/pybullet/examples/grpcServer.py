import pybullet as p
import time

useDirect = False
usePort = True

import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
id = p.loadPlugin("grpcPlugin")
#dynamically loading the plugin
#id = p.loadPlugin("E:/develop/bullet3/bin/pybullet_grpcPlugin_vs2010_x64_debug.dll", postFix="_grpcPlugin")

#start the GRPC server at hostname, port
if (id < 0):
  print("Cannot load grpcPlugin")
  exit(0)

if usePort:
  p.executePluginCommand(id, "localhost:12345")
else:
  p.executePluginCommand(id, "localhost")

while p.isConnected():
  if (useDirect):
    #Only in DIRECT mode, since there is no 'ping' you need to manually call to handle RCPs:
    numRPC = 10
    p.executePluginCommand(id, intArgs=[numRPC])
  else:
    dt = 1. / 240.
    time.sleep(dt)
