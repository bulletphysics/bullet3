import pybullet as p
import time

p.connect(p.SHARED_MEMORY)
timestr = time.strftime("%Y%m%d-%H%M%S")
filename = "saveWorld" + timestr + ".py"
p.saveWorld(filename)
p.disconnect()
