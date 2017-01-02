import pybullet as p
p.connect(p.GUI) #or p.SHARED_MEMORY or p.DIRECT

p.loadURDF("plane.urdf")


p.setGravity(0,0,-10)
resetPosition = [0,0,0.2]

p.loadURDF("phantomx/phantomx.urdf",resetPosition[0],resetPosition[1],resetPosition[2])

for step in range (40000000):
        p.stepSimulation()

p.disconnect()


