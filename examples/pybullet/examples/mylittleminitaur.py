import pybullet as p
import time
import math
import sys
sys.path.append(".")

from minitaur import Minitaur
p.connect(p.GUI)
p.setTimeOut(5)
#p.setPhysicsEngineParameter(numSolverIterations=50)
p.setGravity(0,0,-10)
p.setTimeStep(0.01)

urdfRoot = ''
p.loadURDF("%s/plane.urdf" % urdfRoot)
minitaur = Minitaur(urdfRoot)

while (True):
	p.stepSimulation()
	time.sleep(0.01)
  