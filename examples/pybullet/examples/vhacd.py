import pybullet as p
import pybullet_data as pd
import os

import pybullet_data

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
name_in = os.path.join(pd.getDataPath(), "duck.obj")
name_out = "duck_vhacd.obj"
name_log = "log.txt"
p.vhacd(name_in, name_out, name_log)

