import pybullet as p

p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.loadURDF('plane.urdf', [0, 0, 0], [0, 0, 0, 1])
p.loadURDF('ashcan/ashcan.urdf', [0, 0, 0], [0, 0, 0, 1])

p.loadURDF('birdhouse1/birdhouse1.urdf', [1, -0.2, 0.7], [0, 0, 0, 1])

while True:
	p.stepSimulation()
