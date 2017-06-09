
from envs.bullet.racecarGymEnv import RacecarGymEnv
print ("hello")
environment = RacecarGymEnv(render=True)

targetVelocitySlider = environment._p.addUserDebugParameter("wheelVelocity",-1,1,0)
steeringSlider = environment._p.addUserDebugParameter("steering",-0.5,0.5,0)

while (True):
  targetVelocity = environment._p.readUserDebugParameter(targetVelocitySlider)
  steeringAngle = environment._p.readUserDebugParameter(steeringSlider)
	
  action=[targetVelocity,steeringAngle]
  state, reward, done, info = environment.step(action)
