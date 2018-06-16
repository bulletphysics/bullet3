import pybullet as p
import time
useMaximalCoordinates=False

flags = p.URDF_ENABLE_SLEEPING

p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)

p.loadURDF("plane100.urdf",flags=flags, useMaximalCoordinates=useMaximalCoordinates)
#p.loadURDF("cube_small.urdf", [0,0,0.5], flags=flags)
r2d2 = -1
for k in range (5):
	for i in range (5):
		r2d2=p.loadURDF("r2d2.urdf",[k*2,i*2,1], useMaximalCoordinates=useMaximalCoordinates, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES+flags)

		#enable sleeping: you can pass the flag during URDF loading, or do it afterwards
		#p.changeDynamics(r2d2,-1,activationState=p.ACTIVATION_STATE_ENABLE_SLEEPING)
		
		
		for j in range (p.getNumJoints(r2d2)):
			p.setJointMotorControl2(r2d2,j,p.VELOCITY_CONTROL,targetVelocity=0)
print("r2d2=",r2d2)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
timestep = 1./240.
p.setTimeStep(timestep)
p.setGravity(0,0,-10)

while p.isConnected():
	p.stepSimulation()
	time.sleep(timestep)
	#force the object to wake up
	p.changeDynamics(r2d2,-1,activationState=p.ACTIVATION_STATE_WAKE_UP)
	