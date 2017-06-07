import pybullet as p
import time

cid = p.connect(p.SHARED_MEMORY)
if (cid<0):
	p.connect(p.GUI)
	
p.resetSimulation()
p.setGravity(0,0,-10)

useRealTimeSim = 1

#for video recording (works best on Mac and Linux, not well on Windows)
#p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "racecar.mp4")
p.setRealTimeSimulation(useRealTimeSim) # either this
#p.loadURDF("plane.urdf")
p.loadSDF("stadium.sdf")

car = p.loadURDF("racecar/racecar.urdf")
for i in range (p.getNumJoints(car)):
	print (p.getJointInfo(car,i))

wheels = [2,3,5,7]
steering = [4,6]

targetVelocitySlider = p.addUserDebugParameter("wheelVelocity",-10,10,0)
maxForceSlider = p.addUserDebugParameter("maxForce",0,10,10)
steeringSlider = p.addUserDebugParameter("steering",-0.5,0.5,0)
while (True):
	maxForce = p.readUserDebugParameter(maxForceSlider)
	targetVelocity = p.readUserDebugParameter(targetVelocitySlider)
	steeringAngle = p.readUserDebugParameter(steeringSlider)
	#print(targetVelocity)
	
	for wheel in wheels:
		p.setJointMotorControl2(car,wheel,p.VELOCITY_CONTROL,targetVelocity=targetVelocity,force=maxForce)
		
	for steer in steering:
		p.setJointMotorControl2(car,steer,p.POSITION_CONTROL,targetPosition=steeringAngle)
		
	steering
	if (useRealTimeSim==0):
		p.stepSimulation()
	time.sleep(0.01)
