import pybullet as p
import time
import pybullet_data

p.connect(p.SHARED_MEMORY)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
timestr = time.strftime("%Y%m%d-%H%M%S")
filename = "saveWorld" + timestr + ".py"
p.saveWorld(filename)
p.disconnect()
