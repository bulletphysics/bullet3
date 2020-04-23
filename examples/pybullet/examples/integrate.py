import pybullet as p
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
cube = p.loadURDF("cube.urdf")
frequency = 240
timeStep = 1. / frequency
p.setGravity(0, 0, -9.8)
p.changeDynamics(cube, -1, linearDamping=0, angularDamping=0)
p.setPhysicsEngineParameter(fixedTimeStep=timeStep)
for i in range(frequency):
  p.stepSimulation()
pos, orn = p.getBasePositionAndOrientation(cube)
print(pos)
