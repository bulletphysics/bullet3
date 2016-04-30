import pybullet
pybullet.loadURDF('r2d2.urdf')
pybullet.loadURDF('kuka_lwr/kuka.urdf',3,0,0)

for x in range(0, 1000000):
	pybullet.step()

