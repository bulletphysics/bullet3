import pybullet as p
import pybullet_data

import time
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER,0)

ground = p.loadURDF("plane.urdf",[0,0,0], flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)

p.changeVisualShape(ground,-1,rgbaColor=[1,1,1,0.8])
#p.configureDebugVisualizer(p.COV_ENABLE_PLANAR_REFLECTION,1)

print("hasNumpy = ",p.isNumpyEnabled())


anymal = p.loadURDF("quadruped/ANYmal/robot.urdf",[3,3,3], flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES, useMaximalCoordinates=False)
p.resetSimulation()
ground = p.loadURDF("plane.urdf",[0,0,0], flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)

#todo, tweak this value to trade solver quality versus performance
p.setPhysicsEngineParameter(solverResidualThreshold=1e-2)

index = 0
numX = 10 
numY = 10

for i in range (numX):
	for j in range (numY):
		print("loading animal ", index)
		index+=1
		
		#anymal = p.loadURDF("atlas/atlas_v4_with_multisense.urdf",[i*3,j*3,1], flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
		anymal = p.loadURDF("quadruped/ANYmal/robot.urdf",[(i-numX/2)*2,(j-numY/2)*2,0.6], flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES, useMaximalCoordinates=False)
		
		for j in range(p.getNumJoints(anymal)):
			p.setJointMotorControl2(anymal,j,p.POSITION_CONTROL,targetPosition=0, force=500)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)


p.setGravity(0,0,-10)
#p.setRealTimeSimulation(1)

while (1):
	p.stepSimulation()
	time.sleep(1./240.)
	
		
